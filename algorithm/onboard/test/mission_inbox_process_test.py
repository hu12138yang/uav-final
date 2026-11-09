#!/usr/bin/env python3
"""Exercise delayed KML arrival in the real executable, without any aircraft."""
import os
from pathlib import Path
import pty
import select
import signal
import socket
import subprocess
import sys
import tempfile
import time

binary, config, sample = sys.argv[1:]


def wait_for(log, marker, process, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        log.seek(0)
        text = log.read().decode()
        if marker in text:
            return text
        if process.poll() is not None:
            raise AssertionError(text)
        time.sleep(.02)
    raise AssertionError('Timed out waiting for ' + marker)


with tempfile.TemporaryDirectory() as temp, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
    inbox = Path(temp, 'mission.kml')
    delivery = Path(temp, 'mission.kml.tmp')
    master, slave = pty.openpty()
    udp.bind(('127.0.0.1', 0))
    udp.settimeout(6)
    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen([binary, '--config', config, '--kml', str(inbox),
                                    '--serial', os.ttyname(slave), '--duration', '1.5',
                                    '--uplink', '127.0.0.1:' + str(udp.getsockname()[1])],
                                   stdout=log, stderr=log)
        try:
            wait_for(log, '[STATE] WAIT_KML', process)
            time.sleep(.2)
            assert process.poll() is None
            assert not select.select([master], [], [], 0)[0], 'UART wrote before KML arrival'
            inbox.write_text('<kml>incomplete')
            wait_for(log, 'Expected valid KML XML', process)
            assert process.poll() is None
            assert not select.select([master], [], [], 0)[0], 'Invalid KML activated the UART'
            delivery.write_bytes(Path(sample).read_bytes())
            delivery.replace(inbox)
            wait_for(log, '[KML READY]', process)
            assert udp.recv(8192), 'No status after task accepted'
            inbox.write_text('invalid replacement after acceptance')
            assert process.wait(timeout=6) == 0
            log.seek(0)
            output = log.read().decode()
            assert output.count('[KML READY]') == 1, output
            assert '[OBSERVE]' in output and '[CONTROL]' not in output, output
            print('Real process waits for missing/partial KML, starts on valid delivery, and latches one task')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            os.close(master)
            os.close(slave)

    inbox.unlink()
    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen([binary, '--config', config, '--kml', str(inbox),
                                    '--serial', '/nonexistent-fcu', '--enable-control',
                                    '--livox-config', 'unused.json', '--lidar-ip', '192.168.1.173'],
                                   stdout=log, stderr=log)
        try:
            wait_for(log, '[STATE] WAIT_KML', process)
            delivery.write_bytes(Path(sample).read_bytes())
            delivery.replace(inbox)
            assert process.wait(timeout=5) == 2
            log.seek(0)
            output = log.read().decode()
            assert '[KML READY]' in output and 'calibrated extrinsic.confirmed=true' in output, output
            print('Delayed control start preserves the physical extrinsic guard before any device is opened')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()

    inbox.unlink()
    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen([binary, '--config', config, '--kml', str(inbox),
                                    '--serial', '/nonexistent-fcu'], stdout=log, stderr=log)
        try:
            wait_for(log, '[STATE] WAIT_KML', process)
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=2) == 0
            print('SIGTERM cleanly cancels KML waiting without opening a device')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
    strict = subprocess.run([binary, '--config', config, '--kml', str(inbox), '--prepare-only'],
                            capture_output=True, text=True, timeout=3)
    assert strict.returncode == 2 and 'Cannot open local KML' in strict.stderr, strict
