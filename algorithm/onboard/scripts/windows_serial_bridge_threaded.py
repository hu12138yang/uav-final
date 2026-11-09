#!/usr/bin/env python3
"""Loopback-only temporary Windows COM8 bridge for WSL bench observation."""
import socket,serial,threading,time,ctypes,argparse

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--duration',type=float,default=180);ap.add_argument('--port',default='COM8');args=ap.parse_args()
    stop=threading.Event();counts=[0,0];client=None
    ctypes.windll.winmm.timeBeginPeriod(1)
    try:
      with serial.Serial(args.port,921600,timeout=0.01,write_timeout=0.1) as ser, socket.socket() as listener:
        ser.set_buffer_size(rx_size=65536,tx_size=65536)
        listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);listener.bind(('127.0.0.1',18443));listener.listen(1);listener.settimeout(args.duration)
        print('COM8 threaded bench bridge waiting on 127.0.0.1:18443',flush=True)
        client,_=listener.accept();client.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1);client.settimeout(0.2)
        def from_fcu():
          try:
            while not stop.is_set():
              data=ser.read(ser.in_waiting or 1)
              if data: client.sendall(data);counts[0]+=len(data)
          except (OSError,serial.SerialException): stop.set()
        worker=threading.Thread(target=from_fcu);worker.start();end=time.monotonic()+args.duration
        try:
          while time.monotonic()<end and not stop.is_set():
            try:data=client.recv(16384)
            except socket.timeout:continue
            if not data:break
            ser.write(data);counts[1]+=len(data)
        finally:
          stop.set();worker.join(1);client.close()
        print('Closed: FCU->WSL=%d WSL->FCU=%d bytes'%tuple(counts),flush=True)
    finally: ctypes.windll.winmm.timeEndPeriod(1)
if __name__=='__main__':main()