#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"

// --- FreeRTOS Includes ---
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// ==========================================
// CONFIGURAÇÕES GERAIS E HARDWARE
// ==========================================

// --- Wi-Fi ---
#define WIFI_SSID "Navega+ Highlander 2G"
#define WIFI_PASSWORD "Blessedhr10@"

// --- MQTT ---
#define MQTT_BROKER "broker.emqx.io"
#define MQTT_BROKER_PORT 1883
#define MQTT_TOPIC_HR "embarca/batimentos"
#define MQTT_TOPIC_FALL "embarca/quedas"

// --- I2C Config ---
// MPU6050 (Acelerômetro)
#define I2C_MPU_PORT i2c0
#define SCL_MPU_PIN 0
#define SDA_MPU_PIN 1
#define MPU6050_ADDR 0x68

// MAX30102 (Oxímetro)
#define I2C_MAX_PORT i2c1
#define SDA_MAX_PIN 2
#define SCL_MAX_PIN 3
#define MAX30102_ADDR 0x57

// --- Parâmetros Lógicos ---
#define FREE_FALL_THRESHOLD 0.3f
#define IMPACT_THRESHOLD 2.5f
#define FALL_TIMEOUT_MS 3000

#define DEVICE_ID "SENSOR-PATIENT-001"

// ==========================================
// ESTRUTURAS DE DADOS
// ==========================================

// Mensagem para a fila de MQTT
typedef struct
{
    char topic[64];
    char payload[128];
} MqttMsg_t;

// Dados MPU
typedef struct
{
    float x, y, z;
} VECT_3D;

typedef struct
{
    i2c_inst_t *i2c;
    uint8_t addr;
} MPU6050_t;

typedef struct
{
    float prev_accel_total;
    uint32_t fall_start_time;
    bool free_fall_detected;
    bool impact_detected;
} FallDetector_t;

// Dados MAX30102
typedef struct
{
    float dc_red, dc_ir;
    float acs_prev1; // Valor anterior do sinal AC
    int last_peak;   // Índice do último pico
    float bpm;       // Valor final calculado (suavizado)
    bool finger_on;

    // [CORREÇÃO 1] Buffer melhorado
    float bpm_buf[5];
    int bpm_idx;        // Índice circular
    bool buffer_filled; // Novo: sabe se o buffer já deu uma volta completa

    // [CORREÇÃO 2] Memória para SpO2
    float last_valid_spo2; // Novo: guarda o último valor aceitável
} PulseState;

// ==========================================
// VARIÁVEIS GLOBAIS
// ==========================================
QueueHandle_t mqttQueue;
static mqtt_client_t *mqtt_client;
static ip_addr_t broker_ip;
static bool mqtt_connected = false;

// ==========================================
// DRIVERS I2C / HARDWARE
// ==========================================

// --- MPU6050 Helpers ---
void mpu6050_init_hw(MPU6050_t *mpu, i2c_inst_t *i2c, uint8_t addr)
{
    mpu->i2c = i2c;
    mpu->addr = addr;
    uint8_t buffer[2] = {0x6B, 0x00}; // Wake up
    i2c_write_blocking(mpu->i2c, mpu->addr, buffer, 2, false);
}

void mpu6050_get_accel(MPU6050_t *mpu, VECT_3D *vect)
{
    uint8_t buffer[6];
    uint8_t reg = 0x3B;
    i2c_write_blocking(mpu->i2c, mpu->addr, &reg, 1, true);
    i2c_read_blocking(mpu->i2c, mpu->addr, buffer, 6, false);

    int16_t ax = (buffer[0] << 8) | buffer[1];
    int16_t ay = (buffer[2] << 8) | buffer[3];
    int16_t az = (buffer[4] << 8) | buffer[5];

    vect->x = ax / 16384.0f;
    vect->y = ay / 16384.0f;
    vect->z = az / 16384.0f;
}

// --- MAX30102 Helpers ---
static bool max30102_wr8(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return i2c_write_blocking(I2C_MAX_PORT, MAX30102_ADDR, b, 2, false) == 2;
}

static bool max30102_rd8(uint8_t reg, uint8_t *val)
{
    if (i2c_write_blocking(I2C_MAX_PORT, MAX30102_ADDR, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(I2C_MAX_PORT, MAX30102_ADDR, val, 1, false) == 1;
}

static bool max30102_rdbuf(uint8_t reg, uint8_t *buf, size_t n)
{
    if (i2c_write_blocking(I2C_MAX_PORT, MAX30102_ADDR, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(I2C_MAX_PORT, MAX30102_ADDR, buf, n, false) == (int)n;
}

static bool max_init_hw(void)
{
    max30102_wr8(0x09, 0x40);
    sleep_ms(10); // Reset
    max30102_wr8(0x09, 0x00);
    sleep_ms(10);

    if (!max30102_wr8(0x02, 0x00))
        return false; // Int enable
    if (!max30102_wr8(0x04, 0))
        return false; // FIFO Wr Ptr
    if (!max30102_wr8(0x05, 0))
        return false; // Ovf Counter
    if (!max30102_wr8(0x06, 0))
        return false; // FIFO Rd Ptr
    if (!max30102_wr8(0x08, 0x4F))
        return false; // FIFO config: avg 4, rollover enabled
    if (!max30102_wr8(0x0A, 0x27))
        return false; // Mode: SpO2, 100Hz, 18bit
    if (!max30102_wr8(0x0C, 0x24))
        return false; // LED Red Pulse Amp (Ajustado levemente)
    if (!max30102_wr8(0x0D, 0x24))
        return false; // LED IR Pulse Amp (Ajustado levemente)
    if (!max30102_wr8(0x09, 0x03))
        return false; // Mode control: Multi-LED
    return true;
}

// ==========================================
// LÓGICA DE ALGORITMOS
// ==========================================

// --- Lógica de Queda ---
float calculate_total_accel(VECT_3D *accel)
{
    return sqrt(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
}

void check_fall_logic(FallDetector_t *fd, VECT_3D *accel)
{
    float accel_total = calculate_total_accel(accel);
    MqttMsg_t msg;

    // FASE 1: Queda Livre
    if (accel_total < FREE_FALL_THRESHOLD && !fd->free_fall_detected)
    {
        snprintf(msg.topic, sizeof(msg.topic), "%s", MQTT_TOPIC_FALL);
        snprintf(msg.payload, sizeof(msg.payload), "{\"deviceId\": \"%s\", \"status\": \"queda_livre\", \"g\": %.2f}", DEVICE_ID, accel_total);
        xQueueSend(mqttQueue, &msg, 0);

        fd->free_fall_detected = true;
        fd->fall_start_time = to_ms_since_boot(get_absolute_time());
    }

    // FASE 2: Impacto
    if (fd->free_fall_detected && accel_total > IMPACT_THRESHOLD)
    {
        snprintf(msg.topic, sizeof(msg.topic), "%s", MQTT_TOPIC_FALL);
        int len = snprintf(msg.payload, sizeof(msg.payload),
                           "{\"deviceId\":\"%s\",\"status\":\"impacto\",\"g\":%.2f}",
                           DEVICE_ID, accel_total);
        if (len >= sizeof(msg.payload) - 1)
        {
            printf("ERRO: Payload truncado no impacto! Necessário: %d bytes\n", len);
        }
        printf("Enviando impacto: %s\n", msg.payload);
        xQueueSend(mqttQueue, &msg, 0);

        fd->impact_detected = true;
    }

    // FASE 3: Confirmação
    if (fd->impact_detected)
    {
        uint32_t time_since_fall = to_ms_since_boot(get_absolute_time()) - fd->fall_start_time;

        if (time_since_fall > 500 && time_since_fall < FALL_TIMEOUT_MS)
        {
            if (accel_total > 0.8f && accel_total < 1.2f)
            { // Estabilizado
                snprintf(msg.topic, sizeof(msg.topic), "%s", MQTT_TOPIC_FALL);
                snprintf(msg.payload, sizeof(msg.payload), "{\"status\": \"QUEDA_CONFIRMADA\"}");
                xQueueSend(mqttQueue, &msg, 0);

                memset(fd, 0, sizeof(FallDetector_t));
                fd->prev_accel_total = 1.0f;
            }
        }
        else if (time_since_fall >= FALL_TIMEOUT_MS)
        {
            memset(fd, 0, sizeof(FallDetector_t)); // Timeout
            fd->prev_accel_total = 1.0f;
        }
    }
}

// --- Lógica de Pulso (CORRIGIDA) ---
float process_pulse_logic(PulseState *s, uint32_t red, uint32_t ir, int idx)
{
    const float a_dc = 0.05f;
    s->dc_red += a_dc * ((float)red - s->dc_red);
    s->dc_ir += a_dc * ((float)ir - s->dc_ir);

    float ac_ir = (float)ir - s->dc_ir;
    float ac_red = (float)red - s->dc_red;

    float var_ir = ac_ir * ac_ir;
    float rms_ir = sqrtf(var_ir);

    // Ajuste do limiar de detecção de dedo
    s->finger_on = (s->dc_ir > 5000.0f && s->dc_ir < 240000.0f);

    float calculated_spo2 = 0.0f;

    if (s->finger_on)
    {
        // --- LÓGICA DE BPM (Correção 1) ---
        if (ac_ir > 300 && s->acs_prev1 <= 300)
        {
            float rr = (float)(idx - s->last_peak) / 100.0f; // 100Hz
            s->last_peak = idx;

            if (rr > 0.35 && rr < 1.8)
            { // 33 a 170 BPM
                float instant_bpm = 60.0f / rr;

                // Preenche o buffer circularmente
                s->bpm_buf[s->bpm_idx] = instant_bpm;
                s->bpm_idx = (s->bpm_idx + 1) % 5;

                // Se o índice voltou a 0, o buffer está cheio
                if (s->bpm_idx == 0)
                    s->buffer_filled = true;

                // Calcula a média apenas com valores válidos
                float sum = 0;
                int count = (s->buffer_filled) ? 5 : s->bpm_idx;

                // Se é a primeira amostra, usa ela mesma
                if (count == 0)
                    count = 1;

                for (int k = 0; k < ((s->buffer_filled) ? 5 : s->bpm_idx); k++)
                {
                    sum += s->bpm_buf[k];
                }

                // [CORREÇÃO AQUI]: Usar 'count' em vez de 'valid_samples'
                if (count > 0)
                    s->bpm = sum / count;
            }
        }
        s->acs_prev1 = ac_ir;

        // --- LÓGICA DE SPO2 (Correção 2) ---
        float ratio = (sqrtf(ac_red * ac_red) / s->dc_red) / (rms_ir / s->dc_ir);
        calculated_spo2 = 110.0f - 25.0f * ratio;

        // Limita o teto
        if (calculated_spo2 > 100.0f)
            calculated_spo2 = 100.0f;

        // Filtra quedas bruscas (ruído)
        // Se cair abaixo de 80%, mantém o último valor válido em vez de zerar
        if (calculated_spo2 < 80.0f)
        {
            calculated_spo2 = s->last_valid_spo2;
        }
        else
        {
            s->last_valid_spo2 = calculated_spo2; // Atualiza se for um valor bom
        }
    }
    else
    {
        // Reset suave quando tira o dedo
        s->bpm = 0;
        s->bpm_idx = 0;
        s->buffer_filled = false; // Reinicia o estado do buffer
        s->last_valid_spo2 = 0;   // Reseta SpO2
        return 0;
    }
    return calculated_spo2;
}

// ==========================================
// TAREFAS FREERTOS
// ==========================================

void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    mqtt_connected = (status == MQTT_CONNECT_ACCEPTED);
    printf("MQTT Status: %d\n", status);
}

void vTaskNetwork(void *pvParameters)
{
    if (cyw43_arch_init())
    {
        printf("Falha Wi-Fi init\n");
        vTaskDelete(NULL);
    }
    cyw43_arch_enable_sta_mode();
    printf("Conectando Wi-Fi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0)
    {
        printf("Tentando conectar Wi-Fi...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("Wi-Fi Conectado!\n");

    mqtt_client = mqtt_client_new();
    struct mqtt_connect_client_info_t ci = {.client_id = "pico_rtos_health", .keep_alive = 60};

    ip_addr_t ip_addr;
    while (true)
    {
        err_t err = dns_gethostbyname(MQTT_BROKER, &ip_addr, NULL, NULL);
        if (err == ERR_OK)
            break;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    mqtt_client_connect(mqtt_client, &ip_addr, MQTT_BROKER_PORT, mqtt_connection_cb, NULL, &ci);

    MqttMsg_t msgReceived;

    for (;;)
    {
        if (!mqtt_connected)
        {
            mqtt_client_connect(mqtt_client, &ip_addr, MQTT_BROKER_PORT, mqtt_connection_cb, NULL, &ci);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        if (xQueueReceive(mqttQueue, &msgReceived, pdMS_TO_TICKS(100)) == pdPASS)
        {
            printf("Publicando: %s\n", msgReceived.payload);
            mqtt_publish(mqtt_client, msgReceived.topic, msgReceived.payload, strlen(msgReceived.payload), 0, 0, NULL, NULL);
        }
    }
}

void vTaskMPU(void *pvParameters)
{
    MPU6050_t mpu;
    mpu6050_init_hw(&mpu, I2C_MPU_PORT, MPU6050_ADDR);

    FallDetector_t fd;
    memset(&fd, 0, sizeof(fd));
    fd.prev_accel_total = 1.0f;

    VECT_3D accel;

    for (;;)
    {
        mpu6050_get_accel(&mpu, &accel);
        check_fall_logic(&fd, &accel);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void vTaskMAX(void *pvParameters)
{
    if (!max_init_hw())
    {
        printf("Erro MAX30102\n");
        vTaskDelete(NULL);
    }

    PulseState st;
    memset(&st, 0, sizeof(st));
    st.dc_red = st.dc_ir = 10000.0f;

    int idx = 0;
    MqttMsg_t msg;
    TickType_t last_pub = xTaskGetTickCount();

    for (;;)
    {
        uint8_t wr_ptr, rd_ptr;
        max30102_rd8(0x04, &wr_ptr);
        max30102_rd8(0x06, &rd_ptr);
        int num_samples = (wr_ptr - rd_ptr) & 0x1F;

        if (num_samples > 0)
        {
            uint8_t data[6];
            for (int i = 0; i < num_samples; i++)
            {
                max30102_rdbuf(0x07, data, 6);
                uint32_t red = ((uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | data[2]) & 0x3FFFF;
                uint32_t ir = ((uint32_t)data[3] << 16 | (uint32_t)data[4] << 8 | data[5]) & 0x3FFFF;

                float spo2 = process_pulse_logic(&st, red, ir, idx++);

                // Publica a cada 2 segundos se houver leitura válida
                if (st.finger_on && (xTaskGetTickCount() - last_pub > pdMS_TO_TICKS(2000)))
                {
                    // Verifica se o BPM já estabilizou (média > 40) antes de enviar
                    if (st.bpm > 40.0f)
                    {
                        snprintf(msg.topic, sizeof(msg.topic), "%s", MQTT_TOPIC_HR);
                        snprintf(msg.payload, sizeof(msg.payload), "{\"deviceId\": \"%s\", \"bpm\": %.0f, \"spo2\": %.0f}", DEVICE_ID, st.bpm, spo2);
                        xQueueSend(mqttQueue, &msg, 0);
                        last_pub = xTaskGetTickCount();
                        printf("HR Enviado para %s -> BPM: %.1f SpO2: %.1f\n", DEVICE_ID, st.bpm, spo2);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==========================================
// MAIN
// ==========================================
int main()
{
    stdio_init_all();
    sleep_ms(2000);
    printf("Sistema Iniciado - Versão Corrigida v2\n");

    i2c_init(I2C_MPU_PORT, 400000);
    gpio_set_function(SCL_MPU_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDA_MPU_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_MPU_PIN);
    gpio_pull_up(SDA_MPU_PIN);

    i2c_init(I2C_MAX_PORT, 400000);
    gpio_set_function(SCL_MAX_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDA_MAX_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_MAX_PIN);
    gpio_pull_up(SDA_MAX_PIN);

    mqttQueue = xQueueCreate(10, sizeof(MqttMsg_t));

    xTaskCreate(vTaskNetwork, "Network", 2048, NULL, 1, NULL);
    xTaskCreate(vTaskMPU, "MPU_Task", 1024, NULL, 2, NULL);
    xTaskCreate(vTaskMAX, "MAX_Task", 1024, NULL, 2, NULL);

    vTaskStartScheduler();

    while (1)
        ;
    return 0;
}

// // versao com segurança
// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <math.h>
// #include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"
// #include "hardware/i2c.h"
// #include "lwip/apps/mqtt.h"
// #include "lwip/dns.h"

// // --- FreeRTOS Includes ---
// #include "FreeRTOS.h"
// #include "task.h"
// #include "queue.h"
// #include "semphr.h"

// // ==========================================
// // CONFIGURAÇÕES GERAIS E HARDWARE
// // ==========================================

// // --- Wi-Fi ---
// #define WIFI_SSID "Navega+ Highlander 2G"
// #define WIFI_PASSWORD "Blessedhr10@"

// // --- MQTT ---
// // Adicionado username e password para autenticação básica, conforme recomendado no PDF para protocolos como MQTT
// // Isso melhora a autenticação, um dos princípios básicos de segurança (seção 3.3)
// #define MQTT_BROKER "broker.emqx.io"
// #define MQTT_BROKER_PORT 1883
// #define MQTT_USERNAME "test_user" // Exemplo de username básico
// #define MQTT_PASSWORD "test_pass" // Exemplo de password básico (em produção, use valores seguros e não hardcode)
// #define MQTT_TOPIC_HR "embarca/batimentos"
// #define MQTT_TOPIC_FALL "embarca/quedas"

// // --- I2C Config ---
// // MPU6050 (Acelerômetro)
// #define I2C_MPU_PORT i2c0
// #define SCL_MPU_PIN 0
// #define SDA_MPU_PIN 1
// #define MPU6050_ADDR 0x68

// // MAX30102 (Oxímetro)
// #define I2C_MAX_PORT i2c1
// #define SDA_MAX_PIN 2
// #define SCL_MAX_PIN 3
// #define MAX30102_ADDR 0x57

// // --- Parâmetros Lógicos ---
// #define FREE_FALL_THRESHOLD 0.3f
// #define IMPACT_THRESHOLD 2.5f
// #define FALL_TIMEOUT_MS 3000

// #define DEVICE_ID "SENSOR-PATIENT-001"

// // Chave simples para criptografia XOR básica (confidencialidade básica, seção 3.4)
// // Em produção, use chaves geradas aleatoriamente e gerenciamento de chaves (seção 3.5)
// #define XOR_KEY "simple_key_123"

// // ==========================================
// // ESTRUTURAS DE DADOS
// // ==========================================

// // Mensagem para a fila de MQTT
// typedef struct
// {
//     char topic[64];
//     char payload[256]; // ← Aumente para 256 (ou 512 se quiser folga)
// } MqttMsg_t;

// // Dados MPU
// typedef struct
// {
//     float x, y, z;
// } VECT_3D;

// typedef struct
// {
//     i2c_inst_t *i2c;
//     uint8_t addr;
// } MPU6050_t;

// typedef struct
// {
//     float prev_accel_total;
//     uint32_t fall_start_time;
//     bool free_fall_detected;
//     bool impact_detected;
// } FallDetector_t;

// // Dados MAX30102
// typedef struct
// {
//     float dc_red, dc_ir;
//     float acs_prev1; // Valor anterior do sinal AC
//     int last_peak;   // Índice do último pico
//     float bpm;       // Valor final calculado (suavizado)
//     bool finger_on;

//     // [CORREÇÃO 1] Buffer melhorado
//     float bpm_buf[5];
//     int bpm_idx;        // Índice circular
//     bool buffer_filled; // Novo: sabe se o buffer já deu uma volta completa

//     // [CORREÇÃO 2] Memória para SpO2
//     float last_valid_spo2; // Novo: guarda o último valor aceitável
// } PulseState;

// // ==========================================
// // VARIÁVEIS GLOBAIS
// // ==========================================
// QueueHandle_t mqttQueue;
// static mqtt_client_t *mqtt_client;
// static ip_addr_t broker_ip;
// static bool mqtt_connected = false;

// // ==========================================
// // DRIVERS I2C / HARDWARE
// // ==========================================

// // --- MPU6050 Helpers ---
// void mpu6050_init_hw(MPU6050_t *mpu, i2c_inst_t *i2c, uint8_t addr)
// {
//     mpu->i2c = i2c;
//     mpu->addr = addr;
//     uint8_t buffer[2] = {0x6B, 0x00}; // Wake up
//     i2c_write_blocking(mpu->i2c, mpu->addr, buffer, 2, false);
// }

// void mpu6050_get_accel(MPU6050_t *mpu, VECT_3D *vect)
// {
//     uint8_t buffer[6];
//     uint8_t reg = 0x3B;
//     i2c_write_blocking(mpu->i2c, mpu->addr, &reg, 1, true);
//     i2c_read_blocking(mpu->i2c, mpu->addr, buffer, 6, false);

//     int16_t ax = (buffer[0] << 8) | buffer[1];
//     int16_t ay = (buffer[2] << 8) | buffer[3];
//     int16_t az = (buffer[4] << 8) | buffer[5];

//     vect->x = ax / 16384.0f;
//     vect->y = ay / 16384.0f;
//     vect->z = az / 16384.0f;
// }

// // --- MAX30102 Helpers ---
// static bool max30102_wr8(uint8_t reg, uint8_t val)
// {
//     uint8_t b[2] = {reg, val};
//     return i2c_write_blocking(I2C_MAX_PORT, MAX30102_ADDR, b, 2, false) == 2;
// }

// static bool max30102_rd8(uint8_t reg, uint8_t *val)
// {
//     if (i2c_write_blocking(I2C_MAX_PORT, MAX30102_ADDR, &reg, 1, true) != 1)
//         return false;
//     return i2c_read_blocking(I2C_MAX_PORT, MAX30102_ADDR, val, 1, false) == 1;
// }

// static bool max30102_rdbuf(uint8_t reg, uint8_t *buf, size_t n)
// {
//     if (i2c_write_blocking(I2C_MAX_PORT, MAX30102_ADDR, &reg, 1, true) != 1)
//         return false;
//     return i2c_read_blocking(I2C_MAX_PORT, MAX30102_ADDR, buf, n, false) == (int)n;
// }

// static bool max_init_hw(void)
// {
//     max30102_wr8(0x09, 0x40);
//     sleep_ms(10); // Reset
//     max30102_wr8(0x09, 0x00);
//     sleep_ms(10);

//     if (!max30102_wr8(0x02, 0x00))
//         return false; // Int enable
//     if (!max30102_wr8(0x04, 0))
//         return false; // FIFO Wr Ptr
//     if (!max30102_wr8(0x05, 0))
//         return false; // Ovf Counter
//     if (!max30102_wr8(0x06, 0))
//         return false; // FIFO Rd Ptr
//     if (!max30102_wr8(0x08, 0x4F))
//         return false; // FIFO config: avg 4, rollover enabled
//     if (!max30102_wr8(0x0A, 0x27))
//         return false; // Mode: SpO2, 100Hz, 18bit
//     if (!max30102_wr8(0x0C, 0x24))
//         return false; // LED Red Pulse Amp (Ajustado levemente)
//     if (!max30102_wr8(0x0D, 0x24))
//         return false; // LED IR Pulse Amp (Ajustado levemente)
//     if (!max30102_wr8(0x09, 0x03))
//         return false; // Mode control: Multi-LED
//     return true;
// }

// // ==========================================
// // FUNÇÕES DE SEGURANÇA BÁSICA
// // ==========================================

// // Função simples de criptografia XOR para confidencialidade básica (simétrica simples, seção 3.4)
// // Nota: Isso é muito básico e não substitui AES; use para demonstração. Em produção, use bibliotecas criptográficas reais.
// void xor_encrypt_decrypt(char *data, size_t len)
// {
//     size_t key_len = strlen(XOR_KEY);
//     for (size_t i = 0; i < len; ++i)
//     {
//         data[i] ^= XOR_KEY[i % key_len];
//     }
// }

// // ==========================================
// // LÓGICA DE ALGORITMOS
// // ==========================================

// // --- Lógica de Queda ---
// float calculate_total_accel(VECT_3D *accel)
// {
//     return sqrt(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
// }

// void check_fall_logic(FallDetector_t *fd, VECT_3D *accel)
// {
//     float accel_total = calculate_total_accel(accel);
//     MqttMsg_t msg;

//     // FASE 1: Queda Livre
//     if (accel_total < FREE_FALL_THRESHOLD && !fd->free_fall_detected)
//     {
//         snprintf(msg.topic, sizeof(msg.topic), "%s", MQTT_TOPIC_FALL);
//         snprintf(msg.payload, sizeof(msg.payload),
//                  "{\"deviceId\":\"%s\",\"status\":\"queda_livre\",\"g\":%.2f}",
//                  DEVICE_ID, accel_total);
//         xor_encrypt_decrypt(msg.payload, strlen(msg.payload));
//         xQueueSend(mqttQueue, &msg, 0);

//         fd->free_fall_detected = true;
//         fd->fall_start_time = to_ms_since_boot(get_absolute_time());
//     }

//     // FASE 2: Impacto
//     if (fd->free_fall_detected && accel_total > IMPACT_THRESHOLD)
//     {
//         snprintf(msg.topic, sizeof(msg.topic), "%s", MQTT_TOPIC_FALL);
//         snprintf(msg.payload, sizeof(msg.payload),
//                  "{\"deviceId\":\"%s\",\"status\":\"impacto\",\"g\":%.2f}",
//                  DEVICE_ID, accel_total);
//         xor_encrypt_decrypt(msg.payload, strlen(msg.payload));
//         xQueueSend(mqttQueue, &msg, 0);

//         fd->impact_detected = true;
//     }

//     // FASE 3: Confirmação
//     if (fd->impact_detected)
//     {
//         uint32_t time_since_fall = to_ms_since_boot(get_absolute_time()) - fd->fall_start_time;

//         if (time_since_fall > 500 && time_since_fall < FALL_TIMEOUT_MS)
//         {
//             if (accel_total > 0.8f && accel_total < 1.2f)
//             { // Estabilizado
//                 snprintf(msg.topic, sizeof(msg.topic), "%s", MQTT_TOPIC_FALL);
//                 snprintf(msg.payload, sizeof(msg.payload),
//                          "{\"deviceId\":\"%s\",\"status\":\"QUEDA_CONFIRMADA\",\"g\":%.2f,\"durationMs\":%u,\"totalAccel\":%.2f}",
//                          DEVICE_ID, accel_total, time_since_fall, accel_total);
//                 xor_encrypt_decrypt(msg.payload, strlen(msg.payload));
//                 xQueueSend(mqttQueue, &msg, 0);

//                 memset(fd, 0, sizeof(FallDetector_t));
//                 fd->prev_accel_total = 1.0f;
//             }
//         }
//         else if (time_since_fall >= FALL_TIMEOUT_MS)
//         {
//             memset(fd, 0, sizeof(FallDetector_t)); // Timeout
//             fd->prev_accel_total = 1.0f;
//         }
//     }
// }

// // --- Lógica de Pulso (CORRIGIDA) ---
// float process_pulse_logic(PulseState *s, uint32_t red, uint32_t ir, int idx)
// {
//     const float a_dc = 0.05f;
//     s->dc_red += a_dc * ((float)red - s->dc_red);
//     s->dc_ir += a_dc * ((float)ir - s->dc_ir);

//     float ac_ir = (float)ir - s->dc_ir;
//     float ac_red = (float)red - s->dc_red;

//     float var_ir = ac_ir * ac_ir;
//     float rms_ir = sqrtf(var_ir);

//     // Ajuste do limiar de detecção de dedo
//     s->finger_on = (s->dc_ir > 5000.0f && s->dc_ir < 240000.0f);

//     float calculated_spo2 = 0.0f;

//     if (s->finger_on)
//     {
//         // --- LÓGICA DE BPM (Correção 1) ---
//         if (ac_ir > 300 && s->acs_prev1 <= 300)
//         {
//             float rr = (float)(idx - s->last_peak) / 100.0f; // 100Hz
//             s->last_peak = idx;

//             if (rr > 0.35 && rr < 1.8)
//             { // 33 a 170 BPM
//                 float instant_bpm = 60.0f / rr;

//                 // Preenche o buffer circularmente
//                 s->bpm_buf[s->bpm_idx] = instant_bpm;
//                 s->bpm_idx = (s->bpm_idx + 1) % 5;

//                 // Se o índice voltou a 0, o buffer está cheio
//                 if (s->bpm_idx == 0)
//                     s->buffer_filled = true;

//                 // Calcula a média apenas com valores válidos
//                 float sum = 0;
//                 int count = (s->buffer_filled) ? 5 : s->bpm_idx;

//                 // Se é a primeira amostra, usa ela mesma
//                 if (count == 0)
//                     count = 1;

//                 for (int k = 0; k < ((s->buffer_filled) ? 5 : s->bpm_idx); k++)
//                 {
//                     sum += s->bpm_buf[k];
//                 }

//                 // [CORREÇÃO AQUI]: Usar 'count' em vez de 'valid_samples'
//                 if (count > 0)
//                     s->bpm = sum / count;
//             }
//         }
//         s->acs_prev1 = ac_ir;

//         // --- LÓGICA DE SPO2 (Correção 2) ---
//         float ratio = (sqrtf(ac_red * ac_red) / s->dc_red) / (rms_ir / s->dc_ir);
//         calculated_spo2 = 110.0f - 25.0f * ratio;

//         // Limita o teto
//         if (calculated_spo2 > 100.0f)
//             calculated_spo2 = 100.0f;

//         // Filtra quedas bruscas (ruído)
//         // Se cair abaixo de 80%, mantém o último valor válido em vez de zerar
//         if (calculated_spo2 < 80.0f)
//         {
//             calculated_spo2 = s->last_valid_spo2;
//         }
//         else
//         {
//             s->last_valid_spo2 = calculated_spo2; // Atualiza se for um valor bom
//         }
//     }
//     else
//     {
//         // Reset suave quando tira o dedo
//         s->bpm = 0;
//         s->bpm_idx = 0;
//         s->buffer_filled = false; // Reinicia o estado do buffer
//         s->last_valid_spo2 = 0;   // Reseta SpO2
//         return 0;
//     }
//     return calculated_spo2;
// }

// // ==========================================
// // TAREFAS FREERTOS
// // ==========================================

// void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
// {
//     mqtt_connected = (status == MQTT_CONNECT_ACCEPTED);
//     printf("MQTT Status: %d\n", status);
// }

// void vTaskNetwork(void *pvParameters)
// {
//     if (cyw43_arch_init())
//     {
//         printf("Falha Wi-Fi init\n");
//         vTaskDelete(NULL);
//     }
//     cyw43_arch_enable_sta_mode();
//     printf("Conectando Wi-Fi...\n");
//     while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0)
//     {
//         printf("Tentando conectar Wi-Fi...\n");
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
//     printf("Wi-Fi Conectado!\n");

//     mqtt_client = mqtt_client_new();

//     // Estrutura corrigida: SEM username e password (lwIP padrão não suporta diretamente)
//     struct mqtt_connect_client_info_t ci = {
//         .client_id = "pico_rtos_health",
//         .keep_alive = 60,
//         // .will_topic = NULL,  // opcional, pode deixar zerado ou NULL
//         // .will_msg   = NULL,
//         // .will_qos   = 0,
//         // .will_retain = 0,
//         // .clean_session = 1   // opcional, dependendo da versão lwIP
//     };

//     ip_addr_t ip_addr;
//     while (true)
//     {
//         err_t err = dns_gethostbyname(MQTT_BROKER, &ip_addr, NULL, NULL);
//         if (err == ERR_OK)
//             break;
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }

//     // Conecta sem credenciais adicionais
//     mqtt_client_connect(mqtt_client, &ip_addr, MQTT_BROKER_PORT, mqtt_connection_cb, NULL, &ci);

//     MqttMsg_t msgReceived;

//     for (;;)
//     {
//         if (!mqtt_connected)
//         {
//             mqtt_client_connect(mqtt_client, &ip_addr, MQTT_BROKER_PORT, mqtt_connection_cb, NULL, &ci);
//             vTaskDelay(pdMS_TO_TICKS(2000));
//             continue;
//         }

//         if (xQueueReceive(mqttQueue, &msgReceived, pdMS_TO_TICKS(100)) == pdPASS)
//         {
//             printf("Publicando: %s\n", msgReceived.payload);
//             mqtt_publish(mqtt_client, msgReceived.topic, msgReceived.payload, strlen(msgReceived.payload), 0, 0, NULL, NULL);
//         }
//     }
// }

// void vTaskMPU(void *pvParameters)
// {
//     MPU6050_t mpu;
//     mpu6050_init_hw(&mpu, I2C_MPU_PORT, MPU6050_ADDR);

//     FallDetector_t fd;
//     memset(&fd, 0, sizeof(fd));
//     fd.prev_accel_total = 1.0f;

//     VECT_3D accel;

//     for (;;)
//     {
//         mpu6050_get_accel(&mpu, &accel);
//         check_fall_logic(&fd, &accel);
//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
// }

// // Função de criptografia XOR (Confidencialidade Básica)
// void encrypt_payload(char *data, const char *key)
// {
//     size_t data_len = strlen(data);
//     size_t key_len = strlen(key);
//     for (size_t i = 0; i < data_len; i++)
//     {
//         data[i] = data[i] ^ key[i % key_len];
//     }
// }

// void vTaskMAX(void *pvParameters)
// {
//     if (!max_init_hw())
//     {
//         printf("Erro MAX30102\n");
//         vTaskDelete(NULL);
//     }

//     PulseState st;
//     memset(&st, 0, sizeof(st));
//     st.dc_red = st.dc_ir = 10000.0f;

//     int idx = 0;
//     MqttMsg_t msg;
//     TickType_t last_pub = xTaskGetTickCount();

//     for (;;)
//     {
//         uint8_t wr_ptr, rd_ptr;
//         max30102_rd8(0x04, &wr_ptr);
//         max30102_rd8(0x06, &rd_ptr);
//         int num_samples = (wr_ptr - rd_ptr) & 0x1F;

//         if (num_samples > 0)
//         {
//             uint8_t data[6];
//             for (int i = 0; i < num_samples; i++)
//             {
//                 max30102_rdbuf(0x07, data, 6);
//                 uint32_t red = ((uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | data[2]) & 0x3FFFF;
//                 uint32_t ir = ((uint32_t)data[3] << 16 | (uint32_t)data[4] << 8 | data[5]) & 0x3FFFF;

//                 float spo2 = process_pulse_logic(&st, red, ir, idx++);

//                 // Publica a cada 2 segundos se houver leitura válida
//                 if (st.finger_on && (xTaskGetTickCount() - last_pub > pdMS_TO_TICKS(2000)))
//                 {
//                     // Verifica se o BPM já estabilizou (média > 40) antes de enviar
//                     if (st.bpm > 40.0f)
//                     {
//                         snprintf(msg.topic, sizeof(msg.topic), "%s", MQTT_TOPIC_HR);
//                         snprintf(msg.payload, sizeof(msg.payload), "{\"deviceId\": \"%s\", \"bpm\": %.0f, \"spo2\": %.0f}", DEVICE_ID, st.bpm, spo2);
//                         xor_encrypt_decrypt(msg.payload, strlen(msg.payload)); // Criptografia básica
//                         xQueueSend(mqttQueue, &msg, 0);
//                         last_pub = xTaskGetTickCount();
//                         printf("HR Enviado para %s -> BPM: %.1f SpO2: %.1f\n", DEVICE_ID, st.bpm, spo2);
//                     }
//                 }
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }

// // ==========================================
// // MAIN
// // ==========================================
// int main()
// {
//     stdio_init_all();
//     sleep_ms(2000);
//     printf("Sistema Iniciado - Versão Corrigida v2 com Segurança Básica\n");

//     i2c_init(I2C_MPU_PORT, 400000);
//     gpio_set_function(SCL_MPU_PIN, GPIO_FUNC_I2C);
//     gpio_set_function(SDA_MPU_PIN, GPIO_FUNC_I2C);
//     gpio_pull_up(SCL_MPU_PIN);
//     gpio_pull_up(SDA_MPU_PIN);

//     i2c_init(I2C_MAX_PORT, 400000);
//     gpio_set_function(SCL_MAX_PIN, GPIO_FUNC_I2C);
//     gpio_set_function(SDA_MAX_PIN, GPIO_FUNC_I2C);
//     gpio_pull_up(SCL_MAX_PIN);
//     gpio_pull_up(SDA_MAX_PIN);

//     mqttQueue = xQueueCreate(10, sizeof(MqttMsg_t));

//     xTaskCreate(vTaskNetwork, "Network", 2048, NULL, 1, NULL);
//     xTaskCreate(vTaskMPU, "MPU_Task", 1024, NULL, 2, NULL);
//     xTaskCreate(vTaskMAX, "MAX_Task", 1024, NULL, 2, NULL);

//     vTaskStartScheduler();

//     while (1)
//         ;
//     return 0;
// }