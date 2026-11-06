/*
 * Laboratorio 6 - Nodo TX: ESP32 + DHT22 + SX1262 (chip simulado) con RadioLib
 * MODO_RADIO 1: usa RadioLib real sobre el chip SX1262 simulado de Wokwi (modo principal).
 * MODO_RADIO 0: respaldo sin chip; solo imprime la trama (el ToA medido no esta disponible).
 */
#include <Arduino.h>
#include <DHT.h>

// ------------------------- CONFIGURACION -------------------------
#define MODO_RADIO         1        // 1 = RadioLib + chip SX1262, 0 = solo imprimir
#define SF_ACTUAL          7        // <<< CAMBIAR: 7 o 12
#define BW_KHZ             125.0f   // ancho de banda
#define CR_DENOM           5        // 5 -> CR 4/5
#define PREAMBULO          8        // simbolos
#define POTENCIA_DBM       14
#define FREC_MHZ           915.0f
#define NODO_ID            1
#define PERIODO_MS         2000
#define DHT_PIN            4
#define DHT_TIPO           DHT22

#if MODO_RADIO
  #include <RadioLib.h>
  #define LORA_NSS    5
  #define LORA_DIO1   34
  #define LORA_RESET  27
  #define LORA_BUSY   33
  SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RESET, LORA_BUSY);   // SCK 18, MISO 19, MOSI 23 (SPI por defecto)
#endif

DHT dht(DHT_PIN, DHT_TIPO);

// ------------------------- TRAMA BINARIA -------------------------
// 8 bytes, little-endian (orden nativo del ESP32). Mismo orden en el RX.
struct __attribute__((packed)) Trama {
  uint8_t  nodo;        // 1 B  identificador del nodo
  uint16_t seq;         // 2 B  numero de secuencia (para medir PDR)
  int16_t  temp_c100;   // 2 B  temperatura x100  (24.70 C -> 2470)
  uint16_t hum_d10;     // 2 B  humedad x10       (61.3 %  -> 613)
  uint8_t  crc8;        // 1 B  CRC-8 (poly 0x07) de los 7 bytes previos
};
static_assert(sizeof(Trama) == 8, "La trama debe medir 8 bytes");

uint16_t seq = 0;

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
  float ts = (float)(1UL << sf) / (bw_khz * 1000.0f);         // segundos
  int de = (sf >= 11 && bw_khz <= 125.0f) ? 1 : 0;            // low data rate optimize
  int cr = cr_denom - 4;                                      // 1..4
  float num = 8.0f * pl - 4.0f * sf + 28.0f + 16.0f;          // +16 por CRC, H = 0
  int bloques = (int)ceilf(num / (4.0f * (sf - 2 * de)));
  int n_carga = 8 + max(bloques * (cr + 4), 0);
  return (PREAMBULO + 4.25f + n_carga) * ts * 1000.0f;
}

void marca_tiempo() {
  uint32_t ms = millis();
  Serial.printf("[%02lu:%02lu:%02lu.%03lu] ", (ms / 3600000UL) % 100, (ms / 60000UL) % 60,
                (ms / 1000UL) % 60, ms % 1000UL);
}

void imprime_hex(const uint8_t* buf, size_t n) {
  Serial.print("HEX: ");
  for (size_t i = 0; i < n; i++) Serial.printf("%02X", buf[i]);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  marca_tiempo();
#if MODO_RADIO
  int estado = radio.begin(FREC_MHZ, BW_KHZ, SF_ACTUAL, CR_DENOM,
                           RADIOLIB_SX126X_SYNC_WORD_PRIVATE, POTENCIA_DBM, PREAMBULO, 0.0f, false);
  if (estado != RADIOLIB_ERR_NONE) {
    Serial.printf("Fallo radio.begin(): codigo %d (revisa el chip SX1262 y las conexiones)\n", estado);
    while (true) delay(1000);
  }
  Serial.printf("Radio SX1262 OK | f=%.1f MHz SF%d BW=%.0f kHz CR=4/%d PRE=%d Ptx=%d dBm | trama=%u B\n",
                FREC_MHZ, SF_ACTUAL, BW_KHZ, CR_DENOM, PREAMBULO, POTENCIA_DBM, (unsigned)sizeof(Trama));
#else
  Serial.printf("TX listo (modo sin radio) | SF%d BW=%.0f kHz CR=4/%d | trama=%u B\n",
                SF_ACTUAL, BW_KHZ, CR_DENOM, (unsigned)sizeof(Trama));
#endif
}

void loop() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    marca_tiempo(); Serial.println("Error de lectura del DHT");
    delay(PERIODO_MS);
    return;
  }

  // 1) Serializacion binaria: enteros escalados, sin strings ni float en el aire
  Trama tr;
  tr.nodo      = NODO_ID;
  tr.seq       = seq++;
  tr.temp_c100 = (int16_t)lroundf(t * 100.0f);
  tr.hum_d10   = (uint16_t)lroundf(h * 10.0f);
  tr.crc8      = crc8((const uint8_t*)&tr, sizeof(Trama) - 1);

  // 2) Equivalente JSON: SOLO para comparar tamanos, NO se transmite
  char json[96];
  int n_json = snprintf(json, sizeof(json), "{\"id\":%u,\"seq\":%u,\"t\":%.1f,\"h\":%.1f}",
                        NODO_ID, tr.seq, t, h);

  // 3) Envio y medicion del tiempo que tarda (tiempo simulado de Wokwi)
  int estado = 0;
  uint32_t t_us = 0;
#if MODO_RADIO
  uint32_t t0 = micros();
  estado = radio.transmit((uint8_t*)&tr, sizeof(Trama));
  t_us = micros() - t0;
  float toa_radiolib = radio.getTimeOnAir(sizeof(Trama)) / 1000.0f;
#endif

  // 4) Bitacora con timestamp, tamanos y ToA (formula, RadioLib y medido)
  marca_tiempo();
  Serial.printf("TX #%u | SF%d | T=%.2f C H=%.1f %% | binario=%u B ToA_formula=%.1f ms",
                tr.seq, SF_ACTUAL, tr.temp_c100 / 100.0f, tr.hum_d10 / 10.0f,
                (unsigned)sizeof(Trama), toa_ms(sizeof(Trama), SF_ACTUAL, BW_KHZ, CR_DENOM));
#if MODO_RADIO
  Serial.printf(" ToA_RadioLib=%.1f ms ToA_medido=%.1f ms estado=%d", toa_radiolib, t_us / 1000.0f, estado);
#endif
  Serial.printf(" | JSON=%d B ToA=%.1f ms\n", n_json, toa_ms(n_json, SF_ACTUAL, BW_KHZ, CR_DENOM));
  imprime_hex((const uint8_t*)&tr, sizeof(Trama));
  delay(PERIODO_MS);
}
