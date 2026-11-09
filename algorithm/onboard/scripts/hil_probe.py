#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HIL 协议探针（只读，不发解锁 / 起飞 / 降落 / 切模式）

目的：在接真飞控做硬件在环之前，先单独验证这几件事：
  1. 串口链路、波特率、飞控心跳是否正常；
  2. HOME_POSITION(242) 到底用哪个 CRC_EXTRA（厂商表写 36，标准是 104）；
  3. GLOBAL_POSITION_INT(33) / ATTITUDE(30) / GPS_RAW_INT(24) 是否在发；
  4. 飞控是否接受**机载电脑**发起的 MISSION_REQUEST_LIST(43)；
  5. 43 -> 44 -> 51 -> 73 -> 47 整条下载链路是否真的能走通；
  6. 飞控里的 mission 是否符合机载的假设：item[0]=22、航点 frame=3、高度 > 0.5 m。

安全说明：
  * 本脚本只发心跳、提频、mission 请求，**不会解锁、不会起飞、不会切模式**；
  * 仍然建议拆桨、飞机固定；
  * 不要和 onboard_main --serial 同时占用同一个串口。

用法：
  python3 hil_probe.py --port /dev/ttyUSB0 --baud 921600
  python3 hil_probe.py --port /dev/ttyUSB0 --home-crc 104
  python3 hil_probe.py --port /dev/ttyUSB0 --no-rates --verbose
  python3 hil_probe.py --selftest

依赖：
  pip3 install pyserial
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


# ============================================================
# CRC-16/MCRF4XX
# ============================================================
def _make_crc_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if (crc & 1) else (crc >> 1)
        table.append(crc & 0xFFFF)
    return table


CRC_TABLE = _make_crc_table()


def crc16(data, crc=0xFFFF):
    for b in data:
        crc = (crc >> 8) ^ CRC_TABLE[(b ^ crc) & 0xFF]
    return crc & 0xFFFF


# ============================================================
# CRC_EXTRA（按《地面站&飞控交互协议 V2.8.4》+ 标准 MAVLink）
# ============================================================
CRC_EXTRA = {
    0: 50,    # HEARTBEAT
    1: 124,   # SYS_STATUS
    2: 137,   # SYSTEM_TIME（实测飞控用标准 137；厂商表写 217）
    20: 214,  # PARAM_REQUEST_READ
    22: 220,  # PARAM_VALUE
    23: 168,  # PARAM_SET
    24: 24,   # GPS_RAW_INT
    30: 39,   # ATTITUDE
    33: 104,  # GLOBAL_POSITION_INT
    42: 28,   # MISSION_CURRENT
    43: 132,  # MISSION_REQUEST_LIST
    44: 221,  # MISSION_COUNT
    45: 232,  # MISSION_CLEAR_ALL
    47: 153,  # MISSION_ACK
    51: 196,  # MISSION_REQUEST_INT
    66: 148,  # REQUEST_DATA_STREAM
    73: 38,   # MISSION_ITEM_INT
    76: 152,  # COMMAND_LONG
    77: 143,  # COMMAND_ACK
    84: 143,  # SET_POSITION_TARGET_LOCAL_NED
    147: 154, # BATTERY_STATUS
    148: 178, # AUTOPILOT_VERSION
    242: 36,  # HOME_POSITION（厂商值；标准是 104）
    245: 130, # EXTENDED_SYS_STATE
    253: 83,  # STATUSTEXT
}


def u8(p, o):
    return p[o] if 0 <= o < len(p) else 0


def u16(p, o):
    return struct.unpack_from('<H', p, o)[0] if 0 <= o and o + 2 <= len(p) else 0


def i32(p, o):
    return struct.unpack_from('<i', p, o)[0] if 0 <= o and o + 4 <= len(p) else 0


def f32(p, o):
    return struct.unpack_from('<f', p, o)[0] if 0 <= o and o + 4 <= len(p) else 0.0


# ============================================================
# 组帧 / 解帧
# ============================================================
def pack_v2(sysid, compid, seq, msgid, payload, extra=None):
    if extra is None:
        extra = CRC_EXTRA[msgid]
    # MAVLink2 头（不含 STX）：len, incompat, compat, seq, sysid, compid, msgid[3]
    hdr = bytes([len(payload) & 0xFF, 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF,
                 msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    crc = crc16(hdr + payload + bytes([extra]))
    return b'\xFD' + hdr + payload + struct.pack('<H', crc)


class Frame:
    __slots__ = ('msgid', 'payload', 'sysid', 'compid', 'version')

    def __init__(self, msgid, payload, sysid, compid, version):
        self.msgid = msgid
        self.payload = payload
        self.sysid = sysid
        self.compid = compid
        self.version = version


class Parser:
    """增量解析 MAVLink1/2，按 CRC_EXTRA 校验。"""

    def __init__(self, extra_overrides=None):
        self.buf = bytearray()
        self.crc_errors = {}
        self.unknown = {}
        self.extra = dict(CRC_EXTRA)
        if extra_overrides:
            self.extra.update(extra_overrides)

    def _bad_crc(self, msgid):
        self.crc_errors[msgid] = self.crc_errors.get(msgid, 0) + 1

    def feed(self, data):
        self.buf.extend(data)
        out = []
        while True:
            # 丢掉 STX 之前的噪声
            while self.buf and self.buf[0] not in (0xFD, 0xFE):
                del self.buf[0]
            if not self.buf:
                break

            if self.buf[0] == 0xFD:  # MAVLink2
                if len(self.buf) < 12:
                    break
                ln = self.buf[1]
                incompat = self.buf[2]
                sig = 13 if (incompat & 0x01) else 0
                total = 10 + ln + 2 + sig
                if len(self.buf) < total:
                    break
                msgid = self.buf[7] | (self.buf[8] << 8) | (self.buf[9] << 16)
                extra = self.extra.get(msgid)
                if extra is None:
                    self.unknown[msgid] = self.unknown.get(msgid, 0) + 1
                    del self.buf[:total]
                    continue
                payload = bytes(self.buf[10:10 + ln])
                crc_rx = self.buf[10 + ln] | (self.buf[11 + ln] << 8)
                crc_calc = crc16(bytes(self.buf[1:10]) + payload + bytes([extra]))
                if crc_rx != crc_calc:
                    self._bad_crc(msgid)
                    del self.buf[:total]
                    continue
                out.append(Frame(msgid, payload, self.buf[5], self.buf[6], 2))
                del self.buf[:total]
            else:  # MAVLink1
                if len(self.buf) < 8:
                    break
                ln = self.buf[1]
                total = 6 + ln + 2
                if len(self.buf) < total:
                    break
                msgid = self.buf[5]
                extra = self.extra.get(msgid)
                if extra is None:
                    self.unknown[msgid] = self.unknown.get(msgid, 0) + 1
                    del self.buf[:total]
                    continue
                payload = bytes(self.buf[6:6 + ln])
                crc_rx = self.buf[6 + ln] | (self.buf[7 + ln] << 8)
                crc_calc = crc16(bytes(self.buf[1:6]) + payload + bytes([extra]))
                if crc_rx != crc_calc:
                    self._bad_crc(msgid)
                    del self.buf[:total]
                    continue
                out.append(Frame(msgid, payload, self.buf[3], self.buf[4], 1))
                del self.buf[:total]
        return out


# ============================================================
# 探针
# ============================================================
CMD_NAME = {
    16: 'WAYPOINT(16)',
    20: 'RTL(20)',
    21: 'LAND(21)',
    22: 'TAKEOFF(22)',
    112: 'DELAY(112)',
    177: 'JUMP(177)',
    178: 'SPEED(178)',
    183: 'PAYLOAD(183)',
    206: 'CAMERA(206)',
}


class Probe:
    def __init__(self, ser, args):
        self.ser = ser
        self.args = args
        self.parser = Parser({242: args.home_crc})
        self.seq = 0

        self.counts = {}
        self.fc_hb = 0
        self.fc_sysid = None
        self.fc_compid = None
        self.armed = False
        self.main_mode = 0
        self.sub_mode = 0
        self.home = None
        self.gpi = 0
        self.att = 0
        self.gps = 0
        self.gps_fix = -1
        self.mission_current = None

        self.mission_count = None
        self.mission_type = None
        self.items = {}
        self.t_last_hb = 0.0

    # ---------------- 发送 ----------------
    def send(self, msgid, payload, extra=None):
        frame = pack_v2(self.args.sysid, self.args.compid, self.seq, msgid, payload, extra)
        self.ser.write(frame)
        self.seq = (self.seq + 1) & 0xFF

    def send_heartbeat(self):
        # type=6 GCS, autopilot=8, base_mode=192, system_status=4, custom_mode=0
        self.send(0, struct.pack('<IBBBBB', 0, 6, 8, 192, 4, 3))
        self.t_last_hb = time.time()

    def send_rate(self, msgid, hz):
        # 协议 2.5.1：req_stream_id 填报文 ID（不是标准 MAVLink 的 stream 分组）
        self.send(66, struct.pack('<HBBBB', hz, self.args.target, 1, msgid, 1))

    # ---------------- 接收 ----------------
    def on_frame(self, f):
        self.counts[f.msgid] = self.counts.get(f.msgid, 0) + 1
        p = f.payload

        if f.msgid == 0:  # HEARTBEAT
            hb_type = u8(p, 4)
            if hb_type != 6:  # 只认飞控心跳
                self.fc_hb += 1
                self.fc_sysid = f.sysid
                self.fc_compid = f.compid
                cm = struct.unpack_from('<I', p, 0)[0] if len(p) >= 4 else 0
                self.main_mode = (cm >> 16) & 0xFF
                self.sub_mode = (cm >> 24) & 0xFF
                self.armed = (u8(p, 6) & 0x80) != 0
        elif f.msgid == 242:  # HOME_POSITION
            self.home = (i32(p, 0) * 1e-7, i32(p, 4) * 1e-7, i32(p, 8) * 1e-3)
        elif f.msgid == 33:   # GLOBAL_POSITION_INT
            self.gpi += 1
        elif f.msgid == 30:   # ATTITUDE
            self.att += 1
        elif f.msgid == 24:   # GPS_RAW_INT
            self.gps += 1
            self.gps_fix = u8(p, 28)
        elif f.msgid == 42:   # MISSION_CURRENT
            self.mission_current = u16(p, 0)
        elif f.msgid == 44:   # MISSION_COUNT
            self.mission_count = u16(p, 0)
            self.mission_type = u8(p, 4)
        elif f.msgid == 73:   # MISSION_ITEM_INT
            seq = u16(p, 28)
            self.items[seq] = {
                'seq': seq,
                'command': u16(p, 30),
                'frame': u8(p, 34),
                'x': i32(p, 16),
                'y': i32(p, 20),
                'z': f32(p, 24),
                'p1': f32(p, 0),
                'p2': f32(p, 4),
                'p3': f32(p, 8),
                'p4': f32(p, 12),
            }

    def pump(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            if time.time() - self.t_last_hb >= 1.0:
                self.send_heartbeat()
            data = self.ser.read(2048)
            if data:
                for f in self.parser.feed(data):
                    self.on_frame(f)
            else:
                time.sleep(0.005)

    def pump_until(self, pred, timeout):
        end = time.time() + timeout
        while time.time() < end:
            self.pump(min(0.05, max(0.0, end - time.time())))
            if pred():
                return True
        return False

    # ---------------- 任务下载 ----------------
    def download_mission(self):
        # 3 次重试 MISSION_REQUEST_LIST(43)
        for _ in range(3):
            self.send(43, struct.pack('<BBB', self.args.target, 1, 0))
            if self.pump_until(lambda: self.mission_count is not None, 2.0):
                break
        if self.mission_count is None:
            return False, '没有收到 MISSION_COUNT(44)'

        n = self.mission_count
        if n == 0:
            return False, '飞控里的 mission 是空的（地面站还没上传）'
        if n > self.args.max_items:
            return False, f'mission 条数 {n} 超过上限 {self.args.max_items}'

        for seq in range(n):
            ok = False
            for _ in range(3):
                self.send(51, struct.pack('<HBBB', seq, self.args.target, 1, 0))
                if self.pump_until(lambda: seq in self.items, 1.0):
                    ok = True
                    break
            if not ok:
                return False, f'第 {seq} 条 MISSION_ITEM_INT(73) 没收到'

        # 收齐后必须回 ACK
        self.send(47, struct.pack('<BBB', self.args.target, 1, 0))
        return True, ''


def print_mission(items):
    print('\n-- 飞控 mission --')
    print('seq   command          frame   lat           lon            alt(m)   p1..p4')
    for seq in sorted(items):
        it = items[seq]
        name = CMD_NAME.get(it['command'], f"CMD({it['command']})")
        print(f"{seq:>3}   {name:<16} {it['frame']:>5}   "
              f"{it['x'] * 1e-7:>13.7f} {it['y'] * 1e-7:>14.7f} {it['z']:>8.2f}   "
              f"{it['p1']:.2f}/{it['p2']:.2f}/{it['p3']:.2f}/{it['p4']:.2f}")


def selftest():
    """不接硬件，验证本脚本的 CRC / 组帧 / 解帧。"""
    p = Parser()
    hb = pack_v2(245, 1, 0, 0, struct.pack('<IBBBBB', 0, 6, 8, 192, 4, 3))
    fs = p.feed(hb)
    assert len(fs) == 1 and fs[0].msgid == 0 and fs[0].sysid == 245
    assert fs[0].payload[4] == 6

    # 坏 CRC 必须被统计出来
    bad = bytearray(hb)
    bad[-1] ^= 0xFF
    p2 = Parser()
    assert not p2.feed(bytes(bad))
    assert p2.crc_errors.get(0, 0) == 1

    # MISSION_ITEM_INT 73：38 字节
    payload = struct.pack('<4fii f HH BBBBBB', 0.0, 0.0, 0.0, 0.0,
                          210000000, 113000000, 30.0, 2, 16, 1, 1, 3, 0, 1, 0)
    assert len(payload) == 38
    p3 = Parser()
    fs = p3.feed(pack_v2(1, 1, 1, 73, payload))
    assert len(fs) == 1 and fs[0].msgid == 73
    assert u16(fs[0].payload, 28) == 2
    assert u16(fs[0].payload, 30) == 16
    assert u8(fs[0].payload, 34) == 3
    assert abs(f32(fs[0].payload, 24) - 30.0) < 1e-6

    # MAVLink1 心跳也能解
    hdr = bytes([9, 0, 1, 1, 0])
    hb1_payload = struct.pack('<IBBBBB', 0, 2, 12, 0xC0, 4, 3)
    crc = crc16(hdr + hb1_payload + bytes([CRC_EXTRA[0]]))
    fs = Parser().feed(b'\xFE' + hdr + hb1_payload + struct.pack('<H', crc))
    assert len(fs) == 1 and fs[0].msgid == 0 and fs[0].version == 1

    print('selftest OK: CRC / V1+V2 解帧 / MISSION_ITEM_INT 字段全部正确')


def main():
    ap = argparse.ArgumentParser(description='HIL 协议探针（只读，不解锁/不起飞）')
    ap.add_argument('--port', help='串口设备，例如 /dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--sysid', type=int, default=245, help='机载电脑 sysid（默认 245）')
    ap.add_argument('--compid', type=int, default=1)
    ap.add_argument('--target', type=int, default=1, help='飞控 sysid（默认 1）')
    ap.add_argument('--home-crc', type=int, default=36, choices=(36, 104),
                    help='HOME_POSITION(242) 的 CRC_EXTRA：厂商表 36，标准 104')
    ap.add_argument('--link-timeout', type=float, default=10.0)
    ap.add_argument('--telemetry-timeout', type=float, default=10.0)
    ap.add_argument('--max-items', type=int, default=2000)
    ap.add_argument('--no-rates', action='store_true', help='不发 REQUEST_DATA_STREAM(66)')
    ap.add_argument('--skip-mission', action='store_true', help='只查链路/遥测，不下载 mission')
    ap.add_argument('--verbose', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return 0

    if serial is None:
        print('[FATAL] 缺少 pyserial：pip3 install pyserial', file=sys.stderr)
        return 2
    if not args.port:
        ap.error('--port 必填（或使用 --selftest）')

    print('== HIL 协议探针（只读）==')
    print(f'串口: {args.port} @ {args.baud}, sysid={args.sysid}, target={args.target}, '
          f'HOME CRC_EXTRA={args.home_crc}')
    print('注意：不会解锁/起飞；仍建议拆桨并固定飞机。\n')

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=1.0)
    except Exception as e:
        print(f'[FATAL] 打开串口失败: {e}', file=sys.stderr)
        return 2

    probe = Probe(ser, args)
    probe.send_heartbeat()

    # 1) 等飞控心跳
    print('[链路] 等飞控心跳 ...')
    if not probe.pump_until(lambda: probe.fc_hb > 0, args.link_timeout):
        print('[FAIL] 没收到飞控心跳')
        print('       检查：接线 TX/RX 是否交叉、波特率、飞控是否上电、'
              '串口是否被 onboard_main 占用')
        ser.close()
        return 1
    print(f'[ OK ] 飞控心跳: sysid={probe.fc_sysid} compid={probe.fc_compid}, '
          f'main={probe.main_mode} sub={probe.sub_mode}, armed={int(probe.armed)}')
    if probe.armed:
        print('[WARN] 飞控当前处于解锁状态！请拆桨，并确认飞机已固定。')

    # 2) 提频
    if not args.no_rates:
        probe.send_rate(33, 50)
        probe.send_rate(30, 100)
        print('[提频] GLOBAL_POSITION_INT(33)->50Hz, ATTITUDE(30)->100Hz')

    # 3) 等遥测
    print('[遥测] 等 HOME / 位置 / 姿态 ...')
    probe.pump_until(lambda: probe.home is not None and probe.gpi > 0 and probe.att > 0,
                     args.telemetry_timeout)
    if probe.home:
        print(f'[ OK ] HOME_POSITION(242): lat={probe.home[0]:.7f}, '
              f'lon={probe.home[1]:.7f}, alt={probe.home[2]:.2f} m')
    else:
        print(f'[FAIL] HOME_POSITION(242) 没解出来（CRC_EXTRA={args.home_crc}）')
        if probe.parser.crc_errors.get(242, 0) > 0:
            other = 104 if args.home_crc == 36 else 36
            print(f'       242 有 {probe.parser.crc_errors[242]} 次 CRC 失败，'
                  f'换 --home-crc {other} 再试')
    print(f'[{" OK " if probe.gpi > 0 else "FAIL"}] GLOBAL_POSITION_INT(33): {probe.gpi} 帧')
    print(f'[{" OK " if probe.att > 0 else "FAIL"}] ATTITUDE(30): {probe.att} 帧')
    print(f'[{" OK " if probe.gps > 0 else "WARN"}] GPS_RAW_INT(24): {probe.gps} 帧, '
          f'fix_type={probe.gps_fix}')

    # 4) 任务下载
    mission_ok = False
    mission_msg = ''
    if not args.skip_mission:
        print('\n[任务] 发送 MISSION_REQUEST_LIST(43) ...')
        mission_ok, mission_msg = probe.download_mission()
        if not mission_ok:
            print(f'[FAIL] 任务下载失败: {mission_msg}')
            if probe.parser.crc_errors.get(43, 0):
                print('       43 有 CRC 错误：检查 CRC_EXTRA=132')
        else:
            print(f'[ OK ] MISSION_COUNT(44): N={probe.mission_count}, '
                  f'mission_type={probe.mission_type}')
            print(f'[ OK ] MISSION_ITEM_INT(73): {len(probe.items)}/{probe.mission_count} 条')
            if probe.mission_current is not None:
                print(f'[信息] MISSION_CURRENT(42): seq={probe.mission_current}')
            print_mission(probe.items)

    # 5) mission 语义检查
    seq0 = probe.items.get(0)
    waypoints = [it for it in probe.items.values() if it['command'] == 16]
    takeoff_ok = bool(seq0 and seq0['command'] == 22 and seq0['frame'] == 3)
    wp_frame_ok = all(it['frame'] == 3 for it in waypoints)
    wp_alt_ok = all(it['z'] > 0.5 for it in waypoints)

    print('\n-- 检查汇总 --')
    checks = [
        ('飞控心跳', probe.fc_hb > 0, '检查接线/波特率/供电'),
        ('HOME_POSITION(242)', probe.home is not None,
         f'CRC_EXTRA={args.home_crc}；失败可换 {"104" if args.home_crc == 36 else "36"}'),
        ('GLOBAL_POSITION_INT(33)', probe.gpi > 0, '飞控位置流'),
        ('ATTITUDE(30)', probe.att > 0, '飞控姿态流'),
    ]
    if not args.skip_mission:
        checks += [
            ('任务下载 43->44', probe.mission_count is not None and probe.mission_count > 0,
             '飞控是否接受机载发起的下载请求'),
            ('任务条目 73', probe.mission_count is not None
             and len(probe.items) == probe.mission_count,
             'N 条是否全部收到'),
            ('mission[0] = 22 起飞', takeoff_ok, '地面站航线第一条必须是起飞且 frame=3'),
            ('航点 frame = 3', wp_frame_ok, '航点必须使用相对高度'),
            ('航点高度 > 0.5 m', wp_alt_ok, '避免 0 高度航点'),
        ]
    failed = 0
    for name, ok, hint in checks:
        if ok:
            print(f'  [ OK ] {name}')
        else:
            failed += 1
            print(f'  [FAIL] {name}    {hint}')
    if probe.gps == 0:
        print('  [WARN] 没收到 GPS_RAW_INT(24)，机载的丢星保护会立即触发')

    # 6) 统计
    if probe.parser.crc_errors:
        print('\n-- CRC 失败统计 --')
        for mid, n in sorted(probe.parser.crc_errors.items()):
            print(f'  msgid={mid}: {n} 次')
    if args.verbose:
        print('\n-- 收到的报文统计 --')
        for mid, n in sorted(probe.counts.items()):
            print(f'  msgid={mid}: {n}')

    ser.close()
    print(f'\n{"全部通过" if failed == 0 else f"存在 {failed} 项失败"}')
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
