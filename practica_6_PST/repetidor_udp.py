#!/usr/bin/env python3
"""Repetidor UDP para el Laboratorio 6: recibe lo que envia el gateway y lo reenvia a varias PC.
Uso:  python repetidor_udp.py --destinos 192.168.0.70,192.168.0.71
      Cada destino es IP o IP:puerto (por defecto 1700). Si una de las PC es la del propio repetidor,
      usa otro puerto para su receptor, por ejemplo 192.168.0.66:1701 y ejecuta alli --puerto 1701.
"""
import argparse, socket, sys, time

ap = argparse.ArgumentParser()
ap.add_argument('--destinos', required=True, help='IP o IP:puerto de las PC de los grupos, separadas por comas (sin espacios)')
ap.add_argument('--puerto', type=int, default=1700, help='puerto donde escucha al gateway')
a = ap.parse_args()

destinos = []
for item in a.destinos.split(','):
    item = item.strip()
    if not item:
        continue
    ip, _, pt = item.partition(':')
    try:
        socket.inet_aton(ip)                      # debe ser una IP numerica, como 192.168.0.70
        destinos.append((ip, int(pt) if pt else 1700))
    except (OSError, ValueError):
        sys.exit(f'Destino no valido: "{item}". Usa la IP real de la PC, por ejemplo 192.168.0.70 o 192.168.0.70:1701')

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', a.puerto))
print(f'Repetidor escuchando UDP {a.puerto}. Reenviando a: {destinos}. Ctrl+C para terminar.')
n_push = n_rx = 0
while True:
    pkt, addr = s.recvfrom(4096)
    if len(pkt) < 4:
        continue
    if pkt[3] == 0:                                   # PUSH_DATA: confirmar al gateway y reenviar
        s.sendto(bytes([pkt[0]]) + pkt[1:3] + b'\x01', addr)
        n_push += 1
        if b'"rxpk"' in pkt:
            n_rx += 1
            for d in destinos:
                try:
                    s.sendto(pkt, d)
                except OSError as e:
                    print('   no se pudo enviar a', d, '->', e)
            print(time.strftime('%H:%M:%S'), 'rxpk reenviado a', len(destinos), 'PC | PUSH_DATA:', n_push, '| con rxpk:', n_rx)
    elif pkt[3] == 2:                                 # PULL_DATA: confirmar al gateway (PULL_ACK)
        s.sendto(bytes([pkt[0]]) + pkt[1:3] + b'\x04', addr)
