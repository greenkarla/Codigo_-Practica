// SX1262 simulado para Wokwi - Laboratorio 6 (LoRa P2P, solo transmision)
// Responde a RadioLib (begin/transmit), calcula el Time-on-Air real a partir de
// SF/BW/CR/preambulo/CRC/longitud y retrasa la interrupcion TxDone ese tiempo.
// No implementa recepcion: el enlace y las metricas RF se simulan en Python.
#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define ESTADO_OK  0x22   // modo STBY_RC; (0x22 & 0x0E) no coincide con los codigos de error de RadioLib

typedef struct {
  pin_t nss, busy, dio1, reset;
  spi_dev_t spi;
  timer_t t_tx;
  uint8_t io[1];        // byte en curso (entra MOSI, sale MISO)
  uint8_t cmd[8];       // opcode + primeros parametros de la transaccion
  uint32_t n;           // bytes ya recibidos en la transaccion
  uint8_t txbuf[256];
  uint32_t txlen;
  uint8_t irq_hi, irq_lo;
  uint8_t ptype;        // 0 = GFSK, 1 = LoRa
  uint8_t sf, bw, cr, ldro;
  uint16_t preambulo;
  uint8_t hdr, plen, crc;
} chip_t;

static const char VERSION[16] = "SX1261 V2D 2D02";   // la cadena que RadioLib espera para el SX1262

static void nss_changed(void *ud, pin_t pin, uint32_t value);
static void spi_done(void *ud, uint8_t *buf, uint32_t count);
static void tx_done(void *ud);

// Posicion (dentro de la transaccion) donde empiezan los datos devueltos por cada comando de lectura
static uint32_t inicio_datos(uint8_t op) {
  switch (op) {
    case 0x1D: return 4;                       // ReadRegister: opcode, addr(2), NOP, datos...
    case 0x1E: return 3;                       // ReadBuffer
    case 0x12: case 0x11: case 0x13: case 0x14:
    case 0x15: case 0x10: case 0x17: return 2; // GetIrqStatus, GetPacketType, GetRxBufferStatus, ...
    default:   return 0xFFFF;                  // comandos de escritura: solo devuelven estado
  }
}

// Byte que el chip pone en MISO mientras el maestro envia el byte numero n
static uint8_t salida(chip_t *c, uint32_t n) {
  uint8_t op = c->cmd[0];
  uint32_t d = inicio_datos(op);
  if (n == 0 || n < d) return ESTADO_OK;
  if (op == 0x12) return (n == 2) ? c->irq_hi : c->irq_lo;
  if (op == 0x11) return c->ptype;
  if (op == 0x1D) {
    uint16_t a = (uint16_t)(((uint16_t)c->cmd[1] << 8) | c->cmd[2]) + (uint16_t)(n - 4);
    if (a >= 0x0320 && a < 0x0330) return (uint8_t)VERSION[a - 0x0320];
  }
  return 0x00;
}

static void armar(chip_t *c) {
  c->io[0] = salida(c, c->n);
  spi_start(c->spi, c->io, 1);
}

// Time-on-Air (Semtech AN1200.13) en microsegundos
static uint32_t toa_us(chip_t *c, uint32_t pl) {
  static const uint32_t bw_hz[11] = {7810, 15625, 31250, 62500, 125000, 250000, 500000, 0, 10420, 20830, 41670};
  uint32_t bw = (c->bw < 11 && bw_hz[c->bw]) ? bw_hz[c->bw] : 125000;
  int sf = c->sf, de = c->ldro ? 1 : 0;
  int num = 8 * (int)pl - 4 * sf + 28 + 16 * (c->crc ? 1 : 0) - 20 * (c->hdr ? 1 : 0);
  int den = 4 * (sf - 2 * de);
  int bloques = (num > 0) ? (num + den - 1) / den : 0;
  int n_sim = 8 + bloques * ((int)c->cr + 4);
  double ts_us = (double)(1u << sf) * 1e6 / (double)bw;
  return (uint32_t)(((double)c->preambulo + 4.25 + (double)n_sim) * ts_us + 0.5);
}

static void ejecutar(chip_t *c) {
  uint8_t op = c->cmd[0];
  if (op == 0x8A && c->n >= 2) {
    c->ptype = c->cmd[1];
  } else if (op == 0x8B && c->n >= 5) {
    c->sf = c->cmd[1]; c->bw = c->cmd[2]; c->cr = c->cmd[3]; c->ldro = c->cmd[4];
    printf("[sx1262] Modulacion LoRa: SF%u BW_code=0x%02X CR=4/%u LDRO=%u\n", c->sf, c->bw, c->cr + 4, c->ldro);
  } else if (op == 0x8C && c->n >= 7 && c->ptype == 1) {
    c->preambulo = (uint16_t)(((uint16_t)c->cmd[1] << 8) | c->cmd[2]);
    c->hdr = c->cmd[3]; c->plen = c->cmd[4]; c->crc = c->cmd[5];
  } else if (op == 0x02) {                       // ClearIrqStatus: baja la linea DIO1
    c->irq_hi = 0; c->irq_lo = 0;
    pin_write(c->dio1, LOW);
  } else if (op == 0x82) {
    printf("[sx1262] SetRx ignorado: este simulador no implementa recepcion\n");
  } else if (op == 0x83) {                       // SetTx: arranca la transmision
    uint32_t len = c->plen ? c->plen : c->txlen;
    uint32_t t = toa_us(c, len);
    char hex[2 * 256 + 1];
    for (uint32_t i = 0; i < len && i < 256; i++) sprintf(hex + 2 * i, "%02X", c->txbuf[i]);
    hex[2 * (len < 256 ? len : 256)] = 0;
    printf("[sx1262] TX %u bytes | SF%u CR=4/%u PRE=%u CRC=%u | ToA=%u.%03u ms\n",
           (unsigned)len, c->sf, c->cr + 4, c->preambulo, c->crc, (unsigned)(t / 1000), (unsigned)(t % 1000));
    printf("[sx1262] HEX: %s\n", hex);
    timer_start(c->t_tx, t, false);              // TxDone llega despues del ToA
    c->txlen = 0;
  }
}

static void nss_changed(void *ud, pin_t pin, uint32_t value) {
  chip_t *c = (chip_t *)ud;
  if (value == LOW) {
    c->n = 0;
    memset(c->cmd, 0, sizeof(c->cmd));
    armar(c);
  } else {
    spi_stop(c->spi);
    if (c->n > 0) ejecutar(c);
    c->n = 0;
  }
}

static void spi_done(void *ud, uint8_t *buf, uint32_t count) {
  chip_t *c = (chip_t *)ud;
  if (count == 0 || pin_read(c->nss) != LOW) return;
  uint8_t b = buf[0];
  if (c->n < sizeof(c->cmd)) c->cmd[c->n] = b;
  if (c->cmd[0] == 0x0E && c->n >= 2) {          // WriteBuffer: datos a partir del byte 2
    uint32_t i = (uint32_t)c->cmd[1] + (c->n - 2);
    if (i < sizeof(c->txbuf)) {
      c->txbuf[i] = b;
      if (i + 1 > c->txlen) c->txlen = i + 1;
    }
  }
  c->n++;
  armar(c);
}

static void tx_done(void *ud) {
  chip_t *c = (chip_t *)ud;
  c->irq_lo |= 0x01;                             // IRQ TxDone
  pin_write(c->dio1, HIGH);                      // DIO1 sigue en alto hasta ClearIrqStatus
  printf("[sx1262] TxDone (DIO1 en alto)\n");
}

void chip_init(void) {
  chip_t *c = malloc(sizeof(chip_t));
  memset(c, 0, sizeof(chip_t));
  c->ptype = 1;
  c->preambulo = 8;
  c->nss   = pin_init("NSS", INPUT_PULLUP);
  c->busy  = pin_init("BUSY", OUTPUT_LOW);       // siempre libre
  c->dio1  = pin_init("DIO1", OUTPUT_LOW);
  c->reset = pin_init("RESET", INPUT_PULLUP);

  const spi_config_t cfg = {
    .sck = pin_init("SCK", INPUT), .mosi = pin_init("MOSI", INPUT), .miso = pin_init("MISO", OUTPUT),
    .mode = 0, .done = spi_done, .user_data = c
  };
  c->spi = spi_init((spi_config_t *)&cfg);

  const pin_watch_config_t w = { .edge = BOTH, .pin_change = nss_changed, .user_data = c };
  pin_watch(c->nss, &w);

  const timer_config_t t = { .callback = tx_done, .user_data = c };
  c->t_tx = timer_init(&t);
  printf("[sx1262] Chip SX1262 simulado listo (solo TX)\n");
}
