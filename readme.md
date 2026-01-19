# Sistema de Monitoramento de Saúde com Raspberry Pi Pico W

## Descrição Geral

Este projeto implementa um sistema embarcado de monitoramento de saúde utilizando o Raspberry Pi Pico W. O sistema integra dois sensores principais:
- **MPU6050**: Acelerômetro para detecção de quedas em idosos ou pessoas com mobilidade reduzida.
- **MAX30102**: Sensor de pulso e oxigenação sanguínea (SpO2) para monitoramento de batimentos cardíacos (BPM) e níveis de oxigênio no sangue.

O sistema opera em tempo real com o FreeRTOS, permitindo tarefas paralelas para leitura de sensores, processamento de dados e comunicação de rede. Os dados processados são enviados via MQTT para um broker público (EMQX), permitindo integração com aplicativos remotos para alertas em tempo real (ex.: quedas confirmadas ou métricas vitais).

### Funcionalidades Principais
- **Detecção de Quedas**: Identifica quedas livres, impactos e imobilidade subsequente, enviando alertas via MQTT.
- **Monitoramento Cardíaco**: Calcula BPM e SpO2 em tempo real, enviando dados a cada 2 segundos quando um dedo é detectado no sensor.
- **Comunicação de Rede**: Conexão Wi-Fi, resolução DNS e publicação MQTT em tópicos dedicados.
- **Arquitetura Multitarefa**: Uso de FreeRTOS para tarefas independentes (rede, MPU, MAX), com comunicação via filas.

### Requisitos de Hardware
- Raspberry Pi Pico W (com suporte a Wi-Fi).
- Sensor MPU6050 (acelerômetro) conectado via I2C0 (pinos GPIO 0 e 1).
- Sensor MAX30102 (oxímetro) conectado via I2C1 (pinos GPIO 2 e 3).
- Fonte de alimentação USB ou bateria compatível.
- Rede Wi-Fi com acesso à internet para MQTT.

### Dependências de Software
- SDK do Raspberry Pi Pico (pico-sdk).
- FreeRTOS (incluído via pico-sdk).
- Bibliotecas padrão: stdio, string, stdlib, math.
- Bibliotecas Pico: pico/stdlib, pico/cyw43_arch, hardware/i2c.
- LwIP para rede: lwip/apps/mqtt, lwip/dns.

### Instalação e Configuração
1. **Configurar o Ambiente de Desenvolvimento**:
   - Instale o SDK do Raspberry Pi Pico seguindo as instruções oficiais: [Getting Started with Pico](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html).
   - Ative o suporte a FreeRTOS e LwIP no CMakeLists.txt do projeto.

2. **Compilar e Flashar**:
   - Clone o repositório ou copie o código para um diretório.
   - Crie um build com CMake: `mkdir build && cd build && cmake .. && make`.
   - Conecte o Pico via USB e flash o firmware: `cp *.uf2 /Volumes/RPI-RP2/`.

3. **Configurações Personalizáveis**:
   - Wi-Fi: Altere `WIFI_SSID` e `WIFI_PASSWORD` no código.
   - MQTT: Modifique `MQTT_BROKER`, `MQTT_BROKER_PORT`, `MQTT_TOPIC_HR` e `MQTT_TOPIC_FALL`.
   - Limiares de Queda: Ajuste `FREE_FALL_THRESHOLD` (0.3G), `IMPACT_THRESHOLD` (2.5G) e `FALL_TIMEOUT_MS` (3000ms).
   - Conexões I2C: Verifique os pinos GPIO definidos (ex.: `SCL_MPU_PIN`).

4. **Execução**:
   - Após flashar, conecte um terminal serial (ex.: minicom ou PuTTY) à porta USB do Pico para logs.
   - O sistema inicia automaticamente, conecta ao Wi-Fi e começa a monitorar sensores.

## Estruturas de Dados
As estruturas definem os dados manipulados pelo sistema. Elas são otimizadas para eficiência em memória e uso em tarefas paralelas.

| Estrutura | Campos | Descrição |
|-----------|--------|-----------|
| **MqttMsg_t** | - `char topic[64]`: Nome do tópico MQTT.<br>- `char payload[128]`: Conteúdo JSON da mensagem. | Usada para enfileirar mensagens MQTT entre tarefas. Armazena tópico e payload para envio assíncrono. |
| **VECT_3D** | - `float x, y, z`: Acelerações nos eixos X, Y, Z (em G). | Representa o vetor de aceleração lido do MPU6050. |
| **MPU6050_t** | - `i2c_inst_t *i2c`: Ponteiro para instância I2C.<br>- `uint8_t addr`: Endereço I2C do sensor. | Configuração de hardware para o MPU6050. |
| **FallDetector_t** | - `float prev_accel_total`: Aceleração total anterior.<br>- `uint32_t fall_start_time`: Timestamp do início da queda.<br>- `bool free_fall_detected`: Flag de queda livre.<br>- `bool impact_detected`: Flag de impacto. | Estado da lógica de detecção de quedas, rastreando fases do evento. |
| **PulseState** | - `float dc_red, dc_ir, var_red, var_ir`: Componentes DC e variâncias dos LEDs.<br>- `float ma_sum, ma_buf[5]`: Buffer e soma para média móvel.<br>- `int ma_idx`: Índice no buffer de média móvel.<br>- `int last_peak`: Índice do último pico de pulso.<br>- `float rr_hist[5]`: Histórico de intervalos R-R.<br>- `int rr_len`: Comprimento do histórico.<br>- `float bpm`: Batimentos por minuto calculados.<br>- `bool finger_on`: Flag de dedo detectado.<br>- `float last_rms_ir, last_snr, last_dc_ir`: Métricas de qualidade do sinal IR.<br>- `float acs_prev1, acs_prev2`: Valores anteriores do sinal AC para detecção de picos. | Estado completo para processamento de sinais do MAX30102, incluindo filtros e detecção de picos. |

## Funções Principais
Aqui está uma lista detalhada das funções, incluindo parâmetros, retornos, lógica de negócio e regras.

### Inicialização e Hardware
- **mpu6050_init_hw(MPU6050_t *mpu, i2c_inst_t *i2c, uint8_t addr)**:
  - **Parâmetros**: Ponteiro para estrutura MPU, instância I2C, endereço I2C.
  - **Retorno**: Nenhum (void).
  - **Lógica**: Atribui I2C e endereço à estrutura. Envia comando I2C para acordar o sensor (registro 0x6B = 0x00).
  - **Regras de Negócio**: Garante que o sensor saia do modo de sono. Falha em I2C não é tratada (assume hardware funcional).

- **mpu6050_get_accel(MPU6050_t *mpu, VECT_3D *vect)**:
  - **Parâmetros**: Ponteiro para MPU, ponteiro para vetor de aceleração.
  - **Retorno**: Nenhum (void).
  - **Lógica**: Escreve registro inicial (0x3B), lê 6 bytes via I2C, combina em inteiros 16-bit e converte para G (±2g escala).
  - **Regras de Negócio**: Leitura raw de aceleração. Sensibilidade fixa em 16384 LSB/g.

- **max30102_wr8(uint8_t reg, uint8_t val)**:
  - **Parâmetros**: Registro I2C, valor a escrever.
  - **Retorno**: bool (true se escrita bem-sucedida).
  - **Lógica**: Monta buffer [reg, val] e escreve via I2C.
  - **Regras de Negócio**: Usado para configuração do MAX30102. Verifica se exatamente 2 bytes foram escritos.

- **max30102_rd8(uint8_t reg, uint8_t *val)**:
  - **Parâmetros**: Registro, ponteiro para valor lido.
  - **Retorno**: bool (true se leitura bem-sucedida).
  - **Lógica**: Escreve registro, lê 1 byte.
  - **Regras de Negócio**: Leitura simples de registros de status/control.

- **max30102_rdbuf(uint8_t reg, uint8_t *buf, size_t n)**:
  - **Parâmetros**: Registro inicial, buffer de saída, número de bytes.
  - **Retorno**: bool (true se n bytes lidos).
  - **Lógica**: Escreve registro, lê bloco de n bytes.
  - **Regras de Negócio**: Usado para FIFO de dados (amostras Red/IR).

- **max_init_hw(void)**:
  - **Parâmetros**: Nenhum.
  - **Retorno**: bool (true se inicializado).
  - **Lógica**: Reset via software, zera FIFO/interrupções, configura amostragem (100Hz, 18 bits), correntes LED e modo SpO2.
  - **Regras de Negócio**: Sequência obrigatória para ativar o sensor. Dorme 10ms após reset.

### Lógica de Algoritmos
- **calculate_total_accel(VECT_3D *accel)**:
  - **Parâmetros**: Ponteiro para vetor de aceleração.
  - **Retorno**: float (magnitude em G).
  - **Lógica**: Calcula raiz quadrada da soma dos quadrados (norma euclidiana).
  - **Regras de Negócio**: Representa a aceleração resultante, ignorando direção.

- **check_fall_logic(FallDetector_t *fd, VECT_3D *accel)**:
  - **Parâmetros**: Ponteiro para detector de queda, ponteiro para aceleração.
  - **Retorno**: Nenhum (void).
  - **Lógica de Negócio**:
    - **Fase 1 (Queda Livre)**: Se aceleração < 0.3G e não detectada, envia MQTT {"status": "queda_livre", "g": valor}, marca flag e salva timestamp.
    - **Fase 2 (Impacto)**: Se queda livre detectada e aceleração > 2.5G, envia MQTT {"status": "impacto", "g": valor}, marca impacto.
    - **Fase 3 (Confirmação)**: Após impacto, verifica tempo >500ms e <3000ms. Se aceleração entre 0.8G e 1.2G (imobilidade), envia {"status": "QUEDA_CONFIRMADA"} e reseta estado. Timeout em 3000ms reseta sem alerta.
  - **Regras de Negócio**: Algoritmo em fases para evitar falsos positivos. Baseado em thresholds empíricos; ajustável para sensibilidade.

- **process_pulse_logic(PulseState *s, uint32_t red, uint32_t ir, int idx)**:
  - **Parâmetros**: Ponteiro para estado de pulso, valores Red/IR, índice temporal.
  - **Retorno**: float (SpO2 estimado).
  - **Lógica de Negócio**:
    - Aplica filtro passa-baixa (alpha=0.03) para extrair DC de Red/IR.
    - Calcula AC (oscilação) subtraindo DC.
    - Detecta dedo: DC_IR entre 8000 e 240000.
    - Se dedo presente: Detecta picos (cruzamento de 100), calcula RR (intervalo em seg), filtra 0.4-1.5s (40-150BPM), converte para BPM.
    - Calcula ratio R = (AC_Red/DC_Red) / (RMS_IR/DC_IR), SpO2 = 110 - 25*R (fórmula empírica).
  - **Regras de Negócio**: Processa amostras a 100Hz. Ignora se sem dedo. Filtros evitam ruído; histórico RR não usado aqui (expansível).

### Tarefas FreeRTOS
- **vTaskNetwork(void *pvParameters)**:
  - Inicializa Wi-Fi, conecta à rede, resolve DNS do broker, conecta MQTT.
  - Loop: Recebe mensagens da fila e publica via MQTT. Reconecta se desconectado.

- **vTaskMPU(void *pvParameters)**:
  - Inicializa MPU, loop: Lê aceleração a 20Hz (50ms), processa lógica de queda.

- **vTaskMAX(void *pvParameters)**:
  - Inicializa MAX, loop: Lê FIFO (amostras disponíveis), processa pulso, envia MQTT a cada 2s se dedo detectado.

### Função Principal (main)
- Inicializa stdio, I2C0/I2C1, cria fila MQTT.
- Cria tarefas: Network (pri=1), MPU/MAX (pri=2).
- Inicia escalonador FreeRTOS.

## Regras e Lógica de Negócio Geral
- **Integração de Sensores**: Sensores operam independentemente via tarefas, comunicando via fila MQTT para desacoplamento.
- **Tratamento de Erros**: Mínimo (ex.: deleta tarefa em falha de init). Assume hardware funcional; reconexões automáticas em rede.
- **Eficiência**: Amostragem otimizada (20Hz MPU, 100Hz MAX processado em batch). Uso de floats para precisão, mas otimizável para fixed-point.
- **Segurança**: Não há autenticação MQTT (broker público). Para produção, use TLS e autenticação.
- **Escalabilidade**: Fácil adicionar sensores/tarefas. Lógica de queda/pulso baseada em thresholds empíricos – calibrate com testes reais.

## Contribuição
Contribuições são bem-vindas! Abra issues para bugs ou PRs para melhorias (ex.: filtro Kalman para aceleração, calibração SpO2).

## Licença
MIT License – Use livremente, com atribuição.# HealthHub-IoT-firmware
# HealthHub-IoT-firmware-1
