# Checklist de Melhorias — Sistema LoRa (TX Sensor + RX Gateway)

> Uso sugerido: marque `[x]` conforme implementa e valida cada item em bancada/campo, e faça um commit por item (ou por grupo pequeno de itens relacionados). A ordem dentro de cada bloco reflete a prioridade recomendada. Trechos de código são referência/ponto de partida — ajuste nomes e contexto ao aplicar no arquivo real.

---

## 🔵 Firmware TX (nó sensor)

### Críticos

- [ ] **Controle do pino `DE` do RS485** — adicionar `pinMode(DE, OUTPUT)` e sincronizar com `RE` na troca de direção do barramento.

```cpp
// No setup(), junto com a inicialização de RE:
pinMode(RE, OUTPUT);
digitalWrite(RE, LOW);
pinMode(DE, OUTPUT);
digitalWrite(DE, LOW);

// Em todo ponto onde hoje só existe digitalWrite(RE, HIGH) para transmitir:
digitalWrite(RE, HIGH);
digitalWrite(DE, HIGH);   // habilita transmissão
// ... envia bytes ...
mod.flush();
digitalWrite(RE, LOW);
digitalWrite(DE, LOW);    // volta para modo de recepção
```

- [ ] **Timeout próprio no estado `WAIT`** — evita que o WDT de hardware seja alimentado indefinidamente se o rádio travar.

```cpp
// Variáveis globais novas:
unsigned long waitStartTime = 0;
const unsigned long TX_WAIT_TIMEOUT = 5000; // 5s — ajustar conforme SF/BW usados

// No estado TX, logo após Radio.Send:
case TX:
  timerWrite(timer, 0);
  delay(10);
  debugTx();
  Radio.Send( (uint8_t *)bufferTx, 14);
  waitStartTime = millis();      // inicia contagem do timeout próprio
  state = WAIT;
  break;

// No estado WAIT:
case WAIT:
  timerWrite(timer, 0); // WDT de hardware — protege contra travamento geral do sistema
  Radio.IrqProcess();

  if (millis() - waitStartTime > TX_WAIT_TIMEOUT) {
    Serial.println("Timeout aguardando confirmacao de TX - forcando reset");
    resetModule();
  }
  break;
```

- [ ] **Zeragem dos buffers do Sensor 02** — evita retransmitir dado obsoleto em caso de falha de resposta.

```cpp
void readSensor02(void)
{
  // Zera os buffers antes de nova leitura
  for (uint8_t i = 0; i < 11; i++) bufferAux02[i] = 0x00;
  for (uint8_t i = 0; i < 7;  i++) bufferEC[i]     = 0x00;

  // =========================
  //  Sensor 02
  // =========================
  volatile const uint8_t MS[]     = {0x01, 0x03, 0x00, 0x02, 0x00, 0x02, 0x65, 0xCB};
  volatile const uint8_t CMD_EC[] = {0x01, 0x03, 0x00, 0x15, 0x00, 0x01, 0x95, 0xCE};
  // ... resto da função permanece igual
}
```

### Importantes

- [ ] **Detecção de "sem resposta"** — log quando nenhum byte é recebido.

```cpp
uint8_t c = 0;
while(mod.available()){
  if(c < 9) bufferAux02[c++] = mod.read();
}
if (c == 0) {
  Serial.println("Sensor 02 (T/H): sem resposta");
}

// Replicar o mesmo padrão no bloco de condutividade:
uint8_t Ec = 0;
while(mod.available()){
  if(Ec < 7) bufferEC[Ec++] = mod.read();
}
if (Ec == 0) {
  Serial.println("Sensor 02 (EC): sem resposta");
}
```

- [ ] **Validação de CRC-16 Modbus** nas respostas do Sensor 02.

```cpp
// Função auxiliar de CRC-16 Modbus
uint16_t modbusCRC16(const uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else crc >>= 1;
    }
  }
  return crc;
}

// Ao validar uma resposta de N bytes (ex.: bufferAux02, c bytes recebidos):
if (c >= 5) {
  uint16_t crcRecebido   = ((uint16_t)bufferAux02[c-1] << 8) | bufferAux02[c-2];
  uint16_t crcCalculado  = modbusCRC16((uint8_t*)bufferAux02, c-2);
  if (crcRecebido != crcCalculado) {
    Serial.println("Sensor 02 (T/H): CRC invalido, descartando leitura");
    for (uint8_t i = 0; i < 11; i++) bufferAux02[i] = 0x00;
  }
}
```

- [ ] **Limpeza do buffer serial antes de toda requisição** — hoje só é feito antes do comando de condutividade.

```cpp
// Adicionar também antes do primeiro comando (temperatura/umidade):
while(mod.available()) mod.read();  // limpa lixo de ciclo anterior

digitalWrite(RE, HIGH);
for (uint8_t i=0; i<8; i++) mod.write(MS[i]);
mod.flush();
digitalWrite(RE, LOW);
```

- [ ] **Alinhar `wdtTimeout` com a documentação** — comentário diz 5s, código usa 20000 ms.

```cpp
// Escolher um dos dois e manter consistente:
const int wdtTimeout = 5000;  // 5 segundos, conforme comentário do cabeçalho
// OU atualizar o comentário do cabeçalho para "WDT ativo de 20 segundos"
```

- [ ] **Ajustar `TIME_TO_SLEEP`** para valor real de operação de campo.

```cpp
// Trocar de segundos de teste para minutos reais de operação, ex. 10 minutos:
#define TIME_TO_SLEEP  (10 * 60)   /* 10 minutos, em segundos */
```

- [ ] **Marcar frame fixo do Sensor 01 com aviso de compilação.**

```cpp
void readSensor(void)
{
  #warning "Sensor 01 usando frame de teste - substituir por leitura real antes de ir a campo"
  uint8_t frame[11] = {
    0x01, 0x03, 0x06, 0x02, 0x92, 0xFF, 0x9B, 0x03, 0xE8, 0x38, 0x75
  };
  for(uint8_t i = 0; i<11; i++) bufferAux01[i] = frame[i];
}
```

### Desejáveis / limpeza

- [ ] Remover ou reativar de fato `sdInit()`/gravação em SD no TX.

```cpp
void setup() {
  ...
  // sdInit();   // removido - SD não utilizado neste firmware (TX)
  ...
}
```

- [ ] Remover variáveis mortas: `txCounter`, `lora_idle`, `readStatus`, `dataMessage`.

```cpp
// Remover as declarações não utilizadas:
// volatile uint8_t txCounter = 0;
// bool lora_idle = true;
// bool readStatus = false;
// String dataMessage;
```

- [ ] Corrigir comentário incorreto em `carregaBufferTX()`.

```cpp
void carregaBufferTX(void)
{
  bufferTx[0] = MAC_ADDR;
  bufferTx[1] = SensorADDR;
  // Sensor 1 -> bytes 2 a 7
  for(uint8_t i =2; i < 8; i++) bufferTx[i] = bufferAux01[i+1];
  // Sensor 2 -> bytes 8 a 13   <-- comentário corrigido (era "Sensor 1")
  for(uint8_t i =8; i < 12; i++) bufferTx[i] = bufferAux02[i-5];
  bufferTx[12] = bufferEC[3];
  bufferTx[13] = bufferEC[4];
}
```

- [ ] Renomear `MAC_ADDR` para algo mais claro.

```cpp
// De:
#define MAC_ADDR   0xF5
// Para:
#define CTRL_BYTE_RX   0xF5  // byte de controle esperado pelo RX
// (lembrar de atualizar todas as referências a MAC_ADDR no arquivo)
```

- [ ] Revisar uso de `volatile` fora de contexto real de ISR.

```cpp
// state só é alterado dentro do próprio loop()/callbacks processadas
// via Radio.IrqProcess(), não em ISR de hardware direta.
// Se confirmado, pode simplificar de:
volatile char state = READ_SENSOR;
// para:
char state = READ_SENSOR;
// (manter volatile apenas onde há acesso real por ISR de hardware, ex. resetModule)
```

- [ ] Decidir sobre `counter` (RTC_DATA_ATTR).

```cpp
// Opção A: remover se não fizer sentido manter
// RTC_DATA_ATTR volatile int counter = 0;

// Opção B: transmitir como parte do payload
bufferTx[X] = (uint8_t)counter; // escolher posição X livre no frame, se ampliar o payload
```

---

## 🟠 Firmware RX (gateway)

### Críticos

- [ ] **Trocar `char rxpacket[BUFFER_SIZE]` por `uint8_t rxpacket[BUFFER_SIZE]`** — bug de sinal que corrompe o parsing sempre que um byte recebido for ≥ 0x80.

```cpp
// De:
char rxpacket[BUFFER_SIZE];
// Para:
uint8_t rxpacket[BUFFER_SIZE];

// memcpy continua igual (cópia de bytes):
memcpy(rxpacket, payload, size);

// O parsing em rxManagement() passa a operar sobre valores sempre não-negativos,
// sem necessidade de outras mudanças no restante da lógica de shift/OR.
```

- [ ] **Validar tamanho do pacote recebido antes de parsear.**

```cpp
void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
  esp_task_wdt_reset();
  if (size < 14 || payload[0] != 0xF5) return;   // valida tamanho mínimo + endereço de controle

  memcpy(rxpacket, payload, size);
  Radio.Sleep();
  receivedRSSI = rssi;

  lora_idle = true;
  if (WiFi.status() != WL_CONNECTED) state = WiFi_Recnonnect;
  else state = RX_MANAGEMENT;
}
```

### Importantes (Watchdog / robustez)

- [ ] **Timeout explícito em `getLocalTime()`.**

```cpp
// De:
getLocalTime(&tmstruct);
// Para (timeout de 1s, bem abaixo do WDT de 5000 ms):
getLocalTime(&tmstruct, 1000);
```

- [ ] **Alimentar o watchdog dentro do laço de reconexão WiFi.**

```cpp
case WiFi_Recnonnect:
  esp_task_wdt_reset();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && counter < 7) {
    delay(500);
    esp_task_wdt_reset();   // <-- alimenta o WDT a cada tentativa, não só antes do laço
    Serial.print(".");
    counter++;
  }
  counter = 0;
  Serial.printf("\nWifi status: %d\n", WiFi.status());
  state = RX_MANAGEMENT;
  break;
```

- [ ] **Flag de status do SD (`sdOK`).**

```cpp
bool sdOK = false;   // variável global

void sdInit(void) {
  sd_spi.begin(SCK, MISO, MOSI, CS);
  if (!SD.begin(CS, sd_spi)) {
    Serial.println("Card Mount Failed");
    sdOK = false;
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    sdOK = false;
    return;
  }
  sdOK = true;
  // ... restante da função (prints de tipo/tamanho do cartão) permanece igual
}

// Antes de cada gravação em rxManagement():
if (sdOK) {
  appendFile(SD, "/sensor_01.txt", dataMessage.c_str());
} else {
  Serial.println("SD indisponivel - dado nao gravado em backup local");
}
```

- [ ] **(Opcional) Remontagem periódica do SD.**

```cpp
// Em WAIT, com um contador de tempo próprio (ex. a cada 5 minutos):
static unsigned long lastSdRetry = 0;
if (!sdOK && millis() - lastSdRetry > 5UL * 60UL * 1000UL) {
  Serial.println("Tentando remontar SD...");
  sdInit();
  lastSdRetry = millis();
}
```

### Dados / lógica incompleta

- [ ] **Gravar dados do Sensor 02 no SD.**

```cpp
// Atualizar dataMessage para incluir os campos do Sensor 02:
String dataMessage;
dataMessage = String(timeDay) + "-" + month + "-" + String(timeYear) + "," + hour + ","
            + String(temperature) + "," + String(humidity) + "," + String(conductivity) + ","
            + String(temperature_) + "," + String(humidity_) + "," + String(conductivity_) + ","
            + String(receivedRSSI) + "\r\n";

// E atualizar o cabeçalho do CSV em createFile():
writeFile(SD, path,
  "date, time, Temp1, Umid1, Cond1, Temp2, Umid2, Cond2, RSSI \r\n");
```

- [ ] **Implementar ou remover a lógica de `counterTime`.**

```cpp
// Opção A: remover o bloco morto em WAIT
// if (counterTime == 255 && WiFi.status() == WL_CONNECTED) {
//   getLocalTime(&tmstruct);
//   counterTime = 0;
// }

// Opção B: implementar de fato, com resync independente de RX (ex. a cada 10 min via millis())
static unsigned long lastSync = 0;
if (WiFi.status() == WL_CONNECTED && millis() - lastSync > 10UL * 60UL * 1000UL) {
  getLocalTime(&tmstruct, 1000);
  lastSync = millis();
}
```

- [ ] **Tratar o retorno de `ThingSpeak.writeFields()`.**

```cpp
switch (rxpacket[1]) {
  case ADDR_00:
    x = ThingSpeak.writeFields(channelID_0A, writeAPIKey_0A);
    appendFile(SD, "/sensor_01.txt", dataMessage.c_str());
    break;
  // ... demais casos iguais
}

if (x == 200) {
  Serial.println("Dados enviados com sucesso!");
} else {
  Serial.println("Erro ao enviar ao ThingSpeak: " + String(x));
}
```

### Segurança

- [ ] **Adicionar autenticação ao ElegantOTA.**

```cpp
// De:
ElegantOTA.begin(&server);
// Para:
ElegantOTA.begin(&server, "admin", "sua_senha_forte_aqui");
```

- [ ] **Isolar credenciais em um header separado (`secrets.h`), fora do controle de versão.**

```cpp
// secrets.h (adicionar ao .gitignore!)
#pragma once
#define WIFI_SSID     "TP-Link_2536"
#define WIFI_PASSWORD "Lena1123581321@"

#define TS_CHANNEL_0A 3099475
#define TS_APIKEY_0A  "D1LFNP76QNT6907Z"
#define TS_CHANNEL_0B 2988928
#define TS_APIKEY_0B  "6L03G9YI3V07CBM0"
#define TS_CHANNEL_0C 2989011
#define TS_APIKEY_0C  "8XG6E66W8S7K7OM6"

// No arquivo principal:
#include "secrets.h"
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;
unsigned long channelID_0A = TS_CHANNEL_0A;
const char* writeAPIKey_0A = TS_APIKEY_0A;
// ... etc
// Também remover do histórico as senhas antigas comentadas no código.
```

### Verificações adicionais

- [ ] Confirmar CRC de rádio LoRa habilitado em `LoRaConfig.h` (arquivo não incluso na revisão — checar manualmente).
- [ ] Confirmar escopo real de "plataforma web": só ThingSpeak, ou também dashboard local via `WebServer` (hoje só serve OTA).
- [ ] **(Opcional) Verificação de espaço livre no SD.**

```cpp
uint64_t totalMB = SD.totalBytes() / (1024 * 1024);
uint64_t usedMB  = SD.usedBytes()  / (1024 * 1024);
if (totalMB - usedMB < 10) {  // menos de 10MB livres, por exemplo
  Serial.println("Aviso: cartao SD quase cheio");
}
```

---

## Sugestão de ordem de commits

1. RX: fix `char` → `uint8_t` em `rxpacket` (bug de decodificação)
2. RX: validação de tamanho de pacote em `OnRxDone`
3. TX: controle do pino `DE`
4. TX: timeout próprio no estado `WAIT`
5. TX: zeragem dos buffers do Sensor 02
6. RX: flag `sdOK` + checagem antes de gravar
7. RX: gravação dos dados do Sensor 02 no SD
8. RX: watchdog no laço de reconexão WiFi + timeout do `getLocalTime`
9. RX: autenticação do ElegantOTA + isolamento de credenciais
10. Itens de limpeza/desejáveis (ambos os firmwares)
