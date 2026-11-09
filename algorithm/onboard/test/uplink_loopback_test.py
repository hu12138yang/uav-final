#!/usr/bin/env python3
"""Run the production observe path over a PTY and a real UDP loopback socket."""
import json
import math
import os
from pathlib import Path
import pty
import select
import socket
import struct
import subprocess
import sys
import tempfile
import time


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


def packet(msgid, payload, extra):
    header = bytes([len(payload), 0, 1, 1, msgid])
    return b'\xfe' + header + payload + struct.pack('<H', crc16(header + payload + bytes([extra])))


def check_observe_frames(wire):
    extras = {0: 50, 66: 148, 76: 152, 111: 34}
    ids = set()
    while wire:
        assert wire[0] == 0xfd and len(wire) >= 12, wire.hex()
        size = wire[1] + 12
        frame, wire = wire[:size], wire[size:]
        assert len(frame) == size
        msgid = int.from_bytes(frame[7:10], 'little')
        assert msgid in extras, f'unexpected observe-only message {msgid}'
        assert crc16(frame[1:-2] + bytes([extras[msgid]])) == int.from_bytes(frame[-2:], 'little')
        if msgid == 76:
            payload = frame[10:-2].ljust(33, b'\0')
            assert struct.unpack_from('<H', payload, 28)[0] == 512  # HOME query only
        ids.add(msgid)
    assert {0, 66, 111} <= ids, ids


binary, config, kml = sys.argv[1:]
for flag in ('--sim', '--gps-drop', '--sense-drop'):
    result = subprocess.run([binary, flag], capture_output=True, text=True, timeout=3)
    assert result.returncode == 2 and 'unknown option' in result.stderr, result
result = subprocess.run([binary, '--enable-control', '--config', config, '--kml', kml,
                         '--serial', '/nonexistent-fcu', '--livox-config', 'unused.json',
                         '--lidar-ip', '192.168.1.173'], capture_output=True, text=True, timeout=3)
assert result.returncode == 2 and 'calibrated extrinsic.confirmed=true' in result.stderr, result
print('Removed simulation flags rejected; uncalibrated control blocked before opening devices')
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, tempfile.TemporaryDirectory() as temp:
    receiver.bind(('127.0.0.1', 0))
    port = receiver.getsockname()[1]
    master, slave = pty.openpty()
    args = [binary, '--serial', os.ttyname(slave), '--config', config, '--kml', kml,
            '--duration', '2', '--uplink', f'127.0.0.1:{port}', '--record-dir', temp]
    # Exercise the default observe mode, without --observe-only.
    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen(args, stdout=log, stderr=log)
        try:
            receiver.settimeout(10)
            messages = [json.loads(receiver.recv(8192))]
            heartbeat = packet(0, struct.pack('<IBBBBB', 0, 2, 12, 0, 3, 3), 50)
            gps = bytearray(30)
            gps[28:30] = bytes([6, 20])
            os.write(master, heartbeat + packet(24, gps, 24))
            deadline = time.monotonic() + 10
            while process.poll() is None and time.monotonic() < deadline:
                ready, _, _ = select.select([receiver, master], [], [], .1)
                if receiver in ready:
                    messages.append(json.loads(receiver.recv(8192)))
                if master in ready:
                    os.read(master, 8192)  # Drain TX; recorded copy is checked below.
            assert process.wait(timeout=2) == 0
            expected = {'t', 'phase', 'hold', 'fix', 'est_ok', 'lat', 'lon', 'alt', 'age', 'pos', 'leg', 'clock_ok'}
            assert len(messages) >= 3, messages
            for message in messages:
                assert expected <= message.keys(), message
                assert len(message['pos']) == 3
                assert all(math.isfinite(value) for value in message['pos'])
                assert isinstance(message['est_ok'], bool)
                assert message['phase'] == 'LINK'
            assert any(message['fix'] == 6 for message in messages), messages
            assert all(.35 <= b['t'] - a['t'] <= .75 for a, b in zip(messages, messages[1:])), messages
            check_observe_frames(Path(temp, 'fcu_tx.bin').read_bytes())
            assert Path(temp, 'fcu_rx.bin').read_bytes() == heartbeat + packet(24, gps, 24)
            print('Production observe path: PTY RX/TX, MAVLink CRC/command allowlist, RTK status, UDP JSON at 2 Hz passed')
        except Exception:
            log.seek(0)
            print(log.read().decode(), file=sys.stderr)
            raise
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            os.close(master)
            os.close(slave)

# Verify the real main loop reports link loss instead of running to duration.
master, slave = pty.openpty()
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, tempfile.TemporaryFile() as log:
    receiver.bind(('127.0.0.1', 0))
    receiver.settimeout(10)
    process = subprocess.Popen([binary, '--serial', os.ttyname(slave), '--config', config, '--kml', kml,
                                '--duration', '30', '--uplink', f'127.0.0.1:{receiver.getsockname()[1]}'],
                               stdout=log, stderr=log)
    try:
        receiver.recv(8192)
        os.close(master)
        master = -1
        assert process.wait(timeout=3) == 1
        log.seek(0)
        output = log.read().decode()
        assert 'serial disconnected' in output or 'serial write failed' in output, output
        print('Production main loop: serial disconnect exits with failure passed')
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        if master >= 0:
            os.close(master)
        os.close(slave)
