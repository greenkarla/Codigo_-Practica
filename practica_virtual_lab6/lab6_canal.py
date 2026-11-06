#!/usr/bin/env python3
"""Laboratorio 6 - Canal LoRa simulado + receptor/decodificador (RX).
Uso:
  python3 lab6_canal.py --hex 017b00a609650233 --barrido                 # tabla completa
  python3 lab6_canal.py --hex 017b00a609650233 --dist 800 --sf 7 --n 200
"""
import argparse, math, random, struct

FREC_HZ = 915e6          # banda ISM 902-928 MHz
BW_HZ   = 125e3
PTX_DBM = 14             # potencia de transmision
NF_DB   = 6              # figura de ruido del receptor
SNR_LIM = {7: -7.5, 8: -10, 9: -12.5, 10: -15, 11: -17.5, 12: -20}  # SNR minimo demodulable (dB)

def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c

def toa_ms(pl, sf, bw=BW_HZ, cr=1, preambulo=8, crc=True, explicito=True):
    de = 1 if (sf >= 11 and bw == 125e3) else 0
    ts = (2 ** sf) / bw
    h = 0 if explicito else 1
    num = 8 * pl - 4 * sf + 28 + 16 * (1 if crc else 0) - 20 * h
    n_carga = 8 + max(math.ceil(num / (4 * (sf - 2 * de))) * (cr + 4), 0)
    return (preambulo + 4.25 + n_carga) * ts * 1000

def perdida_db(d, n, sombra):
    fspl_1m = 20 * math.log10(FREC_HZ) - 147.55          # perdida a 1 m
    return fspl_1m + 10 * n * math.log10(max(d, 1)) + sombra

def un_paquete(d, sf, n, sigma):
    rssi = PTX_DBM - perdida_db(d, n, random.gauss(0, sigma))
    ruido = -174 + 10 * math.log10(BW_HZ) + NF_DB
    snr = rssi - ruido
    return rssi, snr, snr >= SNR_LIM[sf]

def desempaquetar(frame):
    if len(frame) != 8 or crc8(frame[:7]) != frame[7]:
        return None
    nodo, seq, t100, h10 = struct.unpack('<BHhH', frame[:7])
    return nodo, seq, t100 / 100.0, h10 / 10.0

def medir(frame, d, sf, n, exp, sigma):
    """Envia n paquetes a d metros con el SF dado. Devuelve (RSSI_prom, SNR_prom, PDR_%, recibidos)."""
    ok = 0; sr = 0.0; ss = 0.0
    for _ in range(n):
        rssi, snr, rx_ok = un_paquete(d, sf, exp, sigma)
        sr += rssi; ss += snr
        if rx_ok and desempaquetar(frame):
            ok += 1
    return sr / n, ss / n, 100.0 * ok / n, ok

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--hex', required=True, help='trama binaria en hexadecimal (la imprime Wokwi)')
    ap.add_argument('--dist', type=float, help='distancia en metros (modo simple)')
    ap.add_argument('--sf', type=int, choices=range(7, 13), help='spreading factor (modo simple)')
    ap.add_argument('--barrido', action='store_true', help='genera la tabla completa (puntos P1..P6 x SF7/SF12)')
    ap.add_argument('--dists', default='100,500,1000,1500,2000,2500', help='distancias del barrido (m)')
    ap.add_argument('--sfs', default='7,12', help='SF del barrido')
    ap.add_argument('--json-len', type=int, default=36, help='bytes del JSON equivalente (barrido)')
    ap.add_argument('--n', type=int, default=200)
    ap.add_argument('--exp', type=float, default=3.5)
    ap.add_argument('--sigma', type=float, default=4.0)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--len', type=int, default=0, help='bytes a usar para el ToA (ej. 36 para el JSON)')
    a = ap.parse_args()
    random.seed(a.seed)
    frame = bytes.fromhex(a.hex)

    if a.barrido:
        dists = [float(x) for x in a.dists.split(',')]
        sfs = [int(x) for x in a.sfs.split(',')]
        d0 = desempaquetar(frame)
        if d0:
            print(f'Trama: nodo={d0[0]} seq={d0[1]} T={d0[2]:.2f}C H={d0[3]:.1f}% ({len(frame)} B) | JSON equivalente: {a.json_len} B')
        print('| Punto | Distancia (m) | SF | Binario (B) | JSON (B) | ToA binario (ms) | ToA JSON (ms) | RSSI (dBm) | SNR (dB) | PDR (%) |')
        print('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |')
        for i, d in enumerate(dists, start=1):
            for sf in sfs:
                random.seed(a.seed)          # misma realizacion de ruido en cada fila: SF7 y SF12 comparables
                rssi, snr, pdr, _ = medir(frame, d, sf, a.n, a.exp, a.sigma)
                print(f'| P{i} | {d:.0f} | {sf} | {len(frame)} | {a.json_len} | {toa_ms(len(frame), sf):.1f} | '
                      f'{toa_ms(a.json_len, sf):.1f} | {rssi:.1f} | {snr:.1f} | {pdr:.1f} |')
        raise SystemExit(0)

    if a.dist is None or a.sf is None:
        ap.error('en modo simple se requieren --dist y --sf (o usa --barrido)')
    L = a.len or len(frame)
    ok = 0; sr = 0.0; ss = 0.0
    for i in range(a.n):
        rssi, snr, rx_ok = un_paquete(a.dist, a.sf, a.exp, a.sigma)
        sr += rssi; ss += snr
        if rx_ok:
            ok += 1
            d = desempaquetar(frame)
            if i < 3 and d:
                print(f'[RX] nodo={d[0]} seq={d[1]} T={d[2]:.2f}C H={d[3]:.1f}% RSSI={rssi:.1f}dBm SNR={snr:.1f}dB')
    print(f'--- d={a.dist:.0f} m SF{a.sf} | bytes={L} | ToA={toa_ms(L, a.sf):.1f} ms '
          f'| RSSI_prom={sr/a.n:.1f} dBm | SNR_prom={ss/a.n:.1f} dB | PDR={100*ok/a.n:.1f}%')
