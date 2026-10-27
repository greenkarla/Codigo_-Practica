#!/usr/bin/env python3
"""Laboratorio 6 (presencial) - Receptor: escucha el packet forwarder (Semtech UDP) del gateway LPS8-V2.
El gateway reenvia a este PC cada trama LoRa que demodula (campos rxpk: rssi, lsnr, datr, data).
Este script filtra tus tramas de 8 bytes (CRC-8 valido), las desempaqueta igual que en la version
virtual y calcula RSSI, SNR y PDR (por numeros de secuencia).

Uso:
  python3 lab6_gateway_rx.py --punto P3 --dist 1000          # mide SF7 y SF12 en una sola pulsacion
"""
import argparse, base64, json, math, socket, struct, sys, time

BW_HZ = 125e3

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

def desempaquetar(frame):
    if len(frame) != 8 or crc8(frame[:7]) != frame[7]:
        return None
    nodo, seq, t100, h10 = struct.unpack('<BHhH', frame[:7])
    return nodo, seq, t100 / 100.0, h10 / 10.0

def parsear_push_data(pkt):
    """Semtech UDP: version(1) token(2) id(1)=0x00 EUI(8) JSON. Devuelve (token, lista_rxpk)."""
    if len(pkt) < 12 or pkt[3] != 0x00:
        return None, []
    token = pkt[1:3]
    try:
        obj = json.loads(pkt[12:].decode('utf-8'))
    except (ValueError, UnicodeDecodeError):
        return token, []
    return token, obj.get('rxpk', [])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--puerto', type=int, default=1700, help='puerto UDP donde el gateway envia (por defecto 1700)')
    ap.add_argument('--punto', default='P?', help='nombre del punto (P1..P6)')
    ap.add_argument('--dist', type=float, default=0, help='distancia gateway-nodo en metros (la mides en el mapa)')
    ap.add_argument('--sfs', default='7,12', help='SF que transmite el nodo, en orden (por defecto 7,12)')
    ap.add_argument('--n', type=int, default=20, help='paquetes por ronda (igual que N_PAQUETES del sketch)')
    ap.add_argument('--nodo', type=int, default=1, help='NODO_ID de tu sketch')
    ap.add_argument('--json-len', type=int, default=36, help='bytes del JSON equivalente')
    ap.add_argument('--espera', type=float, default=0, help='segundos maximos tras el primer paquete (0 = automatico)')
    a = ap.parse_args()

    sfs = [int(x) for x in a.sfs.split(',')]
    periodo = {sf: (3.0 if sf >= 11 else 2.0) for sf in sfs}
    limite = a.espera or (sum(a.n * periodo[sf] for sf in sfs) + 3 * (len(sfs) - 1) + 20)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('0.0.0.0', a.puerto))
    s.settimeout(1.0)
    print(f'Escuchando en UDP {a.puerto}. Pulsa el boton PRG de la Heltec (SF {a.sfs}, {a.n} paquetes por ronda). Ctrl+C para terminar antes.')

    vistos = {sf: {} for sf in sfs}        # sf -> {seq: (rssi, snr)}
    primero = {}                            # sf -> instante del primer paquete valido
    ajenos = 0                              # tramas que no son nuestras (otros dispositivos LoRaWAN, ruido, CRC malo)
    otro_sf = 0
    t_ini = None

    def ronda_completa(sf):
        v = vistos[sf]
        if len(v) >= a.n:
            return True
        return bool(v) and max(v) >= a.n - 1 and time.time() - primero[sf] > periodo[sf]

    try:
        while True:
            if t_ini and time.time() - t_ini > limite:
                print('Tiempo agotado.')
                break
            if t_ini and all(ronda_completa(sf) for sf in sfs):
                break
            try:
                pkt, addr = s.recvfrom(4096)
            except socket.timeout:
                continue
            token, rxpks = parsear_push_data(pkt)
            if token is None:
                continue
            s.sendto(bytes([pkt[0]]) + token + b'\x01', addr)     # PUSH_ACK
            for r in rxpks:
                try:
                    frame = base64.b64decode(r.get('data', ''))
                except ValueError:
                    ajenos += 1; continue
                d = desempaquetar(frame)
                if not d or d[0] != a.nodo:
                    ajenos += 1; continue
                sf = next((x for x in sfs if r.get('datr') == f'SF{x}BW125'), None)
                if sf is None:
                    otro_sf += 1; continue
                if t_ini is None:
                    t_ini = time.time()
                primero.setdefault(sf, time.time())
                if d[1] not in vistos[sf]:
                    vistos[sf][d[1]] = (float(r['rssi']), float(r['lsnr']))
                print(f"[RX] SF{sf} seq={d[1]:>3} T={d[2]:.2f}C H={d[3]:.1f}% RSSI={r['rssi']} dBm SNR={r['lsnr']} dB {r.get('freq')} MHz")
    except KeyboardInterrupt:
        print('\nInterrumpido: se usan los paquetes recibidos hasta ahora.')

    print(f'\nTramas ajenas ignoradas: {ajenos} | con SF no esperado: {otro_sf}')
    print('| Punto | Distancia (m) | SF | Binario (B) | JSON (B) | ToA binario (ms) | ToA JSON (ms) | RSSI (dBm) | SNR (dB) | PDR (%) |')
    print('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |')
    for sf in sfs:
        v = vistos[sf]
        rec = len(v)
        rssi = sum(x[0] for x in v.values()) / rec if rec else float('nan')
        snr = sum(x[1] for x in v.values()) / rec if rec else float('nan')
        r_txt = f'{rssi:.1f}' if rec else '—'
        s_txt = f'{snr:.1f}' if rec else '—'
        print(f'| {a.punto} | {a.dist:.0f} | {sf} | 8 | {a.json_len} | {toa_ms(8, sf):.1f} | {toa_ms(a.json_len, sf):.1f} | '
              f'{r_txt} | {s_txt} | {100.0 * rec / a.n:.1f} |')
        print(f'  (SF{sf}: recibidos {rec}/{a.n})', file=sys.stderr)

if __name__ == '__main__':
    main()
