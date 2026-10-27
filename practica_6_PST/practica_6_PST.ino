#include <Arduino.h>
#include <SPI.h>
#include <DHT.h>
#include <RadioLib.h>

#define PLACA_V3           1        // 1 = Heltec WiFi LoRa 32 V3 (SX1262), 0 = V2 (SX1276)
#define BW_KHZ             125.0f   // ancho de banda
#define CR_DENOM           5        // 5 -> CR 4/5
#define PREAMBULO          8        // simbolos
#define POTENCIA_DBM       14       // igual que el modelo del canal virtual
#define FREC_MHZ           903.9f   // DEBE ser uno de los 8 canales del gateway (US915 sub-banda 2: 903.9 ... 905.3)
#define SYNC_PUBLICO       0x34     // sync word LoRaWAN: el gateway solo demodula esta
#define NODO_ID         // Colocar su numero de lista para definir el nodo
#define N_PAQUETES         20       // paquetes por ronda
#define PERIODO_SF7_MS     2000     // periodo entre paquetes en SF7
#define PERIODO_SF12_MS    3000     // periodo entre paquetes en SF12 (ToA ~1 s)
#define PAUSA_RONDAS_MS    3000
#define USAR_DHT           1        // 0 = valores fijos 24.7 C / 61.3 % (trama de referencia)
#define DHT_PIN            4
#define DHT_TIPO           DHT22
#define TCXO_V             1.8f     // V3: tension del TCXO del SX1262 (si begin() falla prueba 1.6 o 0.0)

const uint8_t SFS[2] = {7, 12};      // se transmite una ronda con cada SF

#if PLACA_V3
  #define PIN_SCK 9
  #define PIN_MISO 11
  #define PIN_MOSI 10
  #define PIN_NSS 8
  #define PIN_DIO1 14
  #define PIN_RST 12
  #define PIN_BUSY 13
  #define PIN_BOTON 0               // boton PRG
  SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);
#else
  #define PIN_SCK 5
  #define PIN_MISO 19
  #define PIN_MOSI 27
  #define PIN_NSS 18
  #define PIN_DIO0 26
  #define PIN_RST 14
  #define PIN_DIO1 35
  #define PIN_BOTON 0
  SX1276 radio = new Module(PIN_NSS, PIN_DIO0, PIN_RST, PIN_DIO1);
#endif

#if USAR_DHT
DHT dht(DHT_PIN, DHT_TIPO);
#endif

// ------------------------- TRAMA BINARIA (identica a la version virtual) -------------------------
struct __attribute__((packed)) Trama {
  uint8_t  nodo;        // 1 B  identificador del nodo
  uint16_t seq;         // 2 B  numero de secuencia (el receptor calcula el PDR con esto)
  int16_t  temp_c100;   // 2 B  temperatura x100
  uint16_t hum_d10;     // 2 B  humedad x10
  uint8_t  crc8;        // 1 B  CRC-8 (poly 0x07) de los 7 bytes previos
};
static_assert(sizeof(Trama) == 8, "La trama debe medir 8 bytes");

uint8_t crc8(const uint8_t* datos, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= datos[i];
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// Time-on-Air (Semtech AN1200.13): cabecera explicita, CRC de payload activo
float toa_ms(uint8_t pl, uint8_t sf, float bw_khz, uint8_t cr_denom) {
  float ts = (float)(1UL << sf) / (bw_khz * 1000.0f);
  int de = (sf >= 11 && bw_khz <= 125.0f) ? 1 : 0;
  int cr = cr_denom - 4;
  float num = 8.0f * pl - 4.0f * sf + 28.0f + 16.0f;
  int bloques = (int)ceilf(num / (4.0f * (sf - 2 * de)));
  int n_carga = 8 + max(bloques * (cr + 4), 0);
  return (PREAMBULO + 4.25f + n_carga) * ts * 1000.0f;
}

void marca_tiempo() {
  uint32_t ms = millis();
  Serial.printf("[%02lu:%02lu:%02lu.%03lu] ", (ms / 3600000UL) % 100, (ms / 60000UL) % 60,
                (ms / 1000UL) % 60, ms % 1000UL);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(PIN_BOTON, INPUT_PULLUP);
#if USAR_DHT
  dht.begin();
#endif
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  marca_tiempo();
#if PLACA_V3
  int estado = radio.begin(FREC_MHZ, BW_KHZ, SFS[0], CR_DENOM, SYNC_PUBLICO, POTENCIA_DBM, PREAMBULO, TCXO_V, false);
#else
  int estado = radio.begin(FREC_MHZ, BW_KHZ, SFS[0], CR_DENOM, SYNC_PUBLICO, POTENCIA_DBM, PREAMBULO);
#endif
  if (estado != RADIOLIB_ERR_NONE) {
    Serial.printf("Fallo radio.begin(): codigo %d (revisa la placa, los pines y el TCXO)\n", estado);
    while (true) delay(1000);
  }
  radio.setCRC(true);
  Serial.printf("Radio OK | f=%.1f MHz BW=%.0f kHz CR=4/%d PRE=%d Ptx=%d dBm sync=0x%02X | trama=%u B | SF7 y SF12\n",
                FREC_MHZ, BW_KHZ, CR_DENOM, PREAMBULO, POTENCIA_DBM, SYNC_PUBLICO, (unsigned)sizeof(Trama));
  Serial.println("Pulsa el boton PRG para enviar las rondas SF7 y SF12.");
}

void ronda(uint8_t sf) {
  int e = radio.setSpreadingFactor(sf);
  if (e != RADIOLIB_ERR_NONE) { marca_tiempo(); Serial.printf("Error al fijar SF%d: %d\n", sf, e); return; }
  const uint32_t periodo = (sf >= 11) ? PERIODO_SF12_MS : PERIODO_SF7_MS;
  marca_tiempo(); Serial.printf("Ronda SF%d: %d paquetes cada %lu ms\n", sf, N_PAQUETES, (unsigned long)periodo);

  for (uint16_t seq = 0; seq < N_PAQUETES; seq++) {
    uint32_t t_inicio = millis();
#if USAR_DHT
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (isnan(t) || isnan(h)) { t = 24.7f; h = 61.3f; }          // sin lectura: valores de referencia
#else
    float t = 24.7f, h = 61.3f;
#endif
    Trama tr;
    tr.nodo      = NODO_ID;
    tr.seq       = seq;
    tr.temp_c100 = (int16_t)lroundf(t * 100.0f);
    tr.hum_d10   = (uint16_t)lroundf(h * 10.0f);
    tr.crc8      = crc8((const uint8_t*)&tr, sizeof(Trama) - 1);

    char json[96];
    int n_json = snprintf(json, sizeof(json), "{\"id\":%u,\"seq\":%u,\"t\":%.1f,\"h\":%.1f}", NODO_ID, tr.seq, t, h);

    uint32_t t0 = micros();
    int estado = radio.transmit((uint8_t*)&tr, sizeof(Trama));     // bloquea hasta terminar de transmitir
    uint32_t t_us = micros() - t0;

    marca_tiempo();
    Serial.printf("TX #%u | SF%d | T=%.2f C H=%.1f %% | binario=%u B ToA_formula=%.1f ms ToA_RadioLib=%.1f ms ToA_medido=%.1f ms estado=%d | JSON=%d B ToA=%.1f ms\n",
                  tr.seq, sf, tr.temp_c100 / 100.0f, tr.hum_d10 / 10.0f, (unsigned)sizeof(Trama),
                  toa_ms(sizeof(Trama), sf, BW_KHZ, CR_DENOM), radio.getTimeOnAir(sizeof(Trama)) / 1000.0f,
                  t_us / 1000.0f, estado, n_json, toa_ms(n_json, sf, BW_KHZ, CR_DENOM));
    Serial.print("HEX: ");
    for (size_t i = 0; i < sizeof(Trama); i++) Serial.printf("%02X", ((const uint8_t*)&tr)[i]);
    Serial.println();

    while (millis() - t_inicio < periodo) delay(5);              // periodo fijo entre paquetes
  }
}

void loop() {
  if (digitalRead(PIN_BOTON) != LOW) { delay(20); return; }      // esperar el boton
  delay(300);                                                     // antirrebote
  for (uint8_t k = 0; k < 2; k++) {
    ronda(SFS[k]);
    if (k == 0) delay(PAUSA_RONDAS_MS);
  }
  marca_tiempo(); Serial.println("Listo. Pulsa PRG para repetir en otro punto.");
}
