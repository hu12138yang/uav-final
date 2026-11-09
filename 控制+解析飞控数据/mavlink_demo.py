# -*- coding: utf-8 -*-
"""
飞控MAVLink串口通信Demo (完整解析版)
功能:
  1. 通过串口连接飞控
  2. 键盘手动控制（虚拟摇杆 MANUAL_CONTROL, ID=69）
  3. 完整解析飞控返回的所有报文数据（19种报文类型）
  4. 指令执行前后对比观察（自动快照）

协议依据：地面站&飞控交互协议 V2.8.4
"""

import serial
import serial.tools.list_ports
import struct
import time
import msvcrt
import sys
import os

# 导入完整解析器
_parser_path = os.path.dirname(os.path.abspath(__file__))
if _parser_path not in sys.path:
    sys.path.insert(0, _parser_path)
import mavlink_parser

# ============================================================
# 1. MAVLink CRC 工具函数
# ============================================================

MAVLINK_CRC_TABLE = mavlink_parser.MAVLINK_CRC_TABLE


def crc_calculate(buf, crc=0xFFFF):
    for b in buf:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        crc = (crc >> 8) ^ MAVLINK_CRC_TABLE[tmp]
    return crc


# ============================================================
# 2. MAVLink 消息 CRC_EXTRA 表
# ============================================================

MSG_CRC_EXTRA = mavlink_parser.MSG_CRC_EXTRA.copy()


# ============================================================
# 3. MAVLink V2 帧编码/解码
# ============================================================

MAVLINK_V2_STX = 0xFD
MAVLINK_V1_STX = 0xFE


class MAVLinkV2Frame:
    def __init__(self):
        self.len = 0
        self.incompat_flags = 0
        self.compat_flags = 0
        self.seq = 0
        self.sysid = 255
        self.compid = 1
        self.msgid = 0
        self.payload = b""
        self.raw = b""

    def pack(self) -> bytes:
        """打包为 MAVLink V2 帧"""
        header = struct.pack('<BBBBBBB', MAVLINK_V2_STX, self.len,
                             self.incompat_flags, self.compat_flags,
                             self.seq, self.sysid, self.compid)
        header += struct.pack('<I', self.msgid)[:3]
        crc_extra = MSG_CRC_EXTRA.get(self.msgid, 0)
        crc = crc_calculate(header[1:] + self.payload)
        crc = crc_calculate(bytes([crc_extra]), crc)
        return header + self.payload + struct.pack('<H', crc)

    def pack_v1(self) -> bytes:
        """打包为 MAVLink V1 帧（兼容性更好，不截断零字节）"""
        header = struct.pack('<BBBBBB', MAVLINK_V1_STX, self.len,
                             self.seq, self.sysid, self.compid, self.msgid & 0xFF)
        crc_extra = MSG_CRC_EXTRA.get(self.msgid, 0)
        crc = crc_calculate(header[1:] + self.payload)
        crc = crc_calculate(bytes([crc_extra]), crc)
        return header + self.payload + struct.pack('<H', crc)

    @staticmethod
    def parse(data: bytes):
        """解析 MAVLink V2 或 V1 帧，返回 (frame, remaining_bytes)"""
        if len(data) < 6:
            return None, data

        # ---- 同时查找 V2 和 V1 帧头，取最先出现的 ----
        v2_idx = -1
        v1_idx = -1
        for i, b in enumerate(data):
            if b == MAVLINK_V2_STX and v2_idx < 0:
                v2_idx = i
            if b == MAVLINK_V1_STX and v1_idx < 0:
                v1_idx = i
            if v2_idx >= 0 and v1_idx >= 0:
                break

        # 先尝试 V2，再尝试 V1
        if v2_idx >= 0:
            frame, rem = MAVLinkV2Frame._try_parse_v2(data, v2_idx)
            if frame:
                return frame, rem

        if v1_idx >= 0:
            frame, rem = MAVLinkV2Frame._try_parse_v1(data, v1_idx)
            if frame:
                return frame, rem

        # 两种都失败：处理数据中没有 MAVLink 帧头的情况
        valid = [v for v in [v2_idx, v1_idx] if v >= 0]
        if valid:
            skip = min(valid)
            return None, data[skip + 1:]
        # 数据中完全没有 MAVLink 帧头（波特率不匹配或噪声），跳过前1个字节
        return None, data[1:]

    @staticmethod
    def _try_parse_v2(data, stx_idx):
        """尝试解析 V2 帧，成功返回 (frame, remaining)"""
        if len(data) < stx_idx + 12:
            return None, data
        header = data[stx_idx:stx_idx + 10]
        _, payload_len, iflags, cflags, seq, sysid, compid = \
            struct.unpack('<BBBBBBB', header[:7])
        msgid = struct.unpack('<I', header[7:10] + b'\x00')[0]
        total_len = 10 + payload_len + 2
        if len(data) < stx_idx + total_len:
            return None, data
        payload = data[stx_idx + 10:stx_idx + 10 + payload_len]
        received_crc = struct.unpack('<H', data[stx_idx + 10 + payload_len:
                                              stx_idx + 10 + payload_len + 2])[0]
        crc_extra = MSG_CRC_EXTRA.get(msgid, 0)
        crc = crc_calculate(header[1:])
        crc = crc_calculate(payload, crc)
        crc = crc_calculate(bytes([crc_extra]), crc)
        if crc != received_crc:
            return None, data[stx_idx + 1:]
        frame = MAVLinkV2Frame()
        frame.len = payload_len; frame.incompat_flags = iflags
        frame.compat_flags = cflags; frame.seq = seq
        frame.sysid = sysid; frame.compid = compid
        frame.msgid = msgid; frame.payload = payload
        frame.raw = data[stx_idx:stx_idx + total_len]
        return frame, data[stx_idx + total_len:]

    @staticmethod
    def _try_parse_v1(data, stx_idx):
        """
        解析 MAVLink V1 帧
        V1: STX(1) | LEN(1) | SEQ(1) | SYS(1) | COMP(1) | MSG(1) | PAYLOAD(LEN) | CRC(2)
        """
        if len(data) < stx_idx + 8:
            return None, data
        payload_len = data[stx_idx + 1]
        total_len = 6 + payload_len + 2   # header=6 + payload + crc2
        if len(data) < stx_idx + total_len:
            return None, data
        seq, sysid, compid, msgid = data[stx_idx+2], data[stx_idx+3], data[stx_idx+4], data[stx_idx+5]
        payload = data[stx_idx + 6:stx_idx + 6 + payload_len]
        received_crc = struct.unpack('<H', data[stx_idx + 6 + payload_len:
                                                stx_idx + 6 + payload_len + 2])[0]

        crc_extra = MSG_CRC_EXTRA.get(msgid, 0)
        crc = crc_calculate(data[stx_idx + 1:stx_idx + 6 + payload_len])  # LEN+SEQ+SYS+COMP+MSG+PAYLOAD
        crc = crc_calculate(bytes([crc_extra]), crc)
        if crc != received_crc:
            return None, data[stx_idx + 1:]
        frame = MAVLinkV2Frame()
        frame.len = payload_len; frame.incompat_flags = 0

        frame.compat_flags = 0; frame.seq = seq
        frame.sysid = sysid; frame.compid = compid
        frame.msgid = msgid; frame.payload = payload
        frame.raw = data[stx_idx:stx_idx + total_len]
        return frame, data[stx_idx + total_len:]


# ============================================================
# 4. 消息构建函数
# ============================================================

_seq_counter = 0

def _next_seq():
    global _seq_counter
    _seq_counter = (_seq_counter + 1) & 0xFF
    return _seq_counter

def build_heartbeat():
    frame = MAVLinkV2Frame()
    frame.msgid = 0
    frame.len = 9
    frame.sysid = 255
    frame.compid = 1
    frame.seq = _next_seq()
    frame.payload = struct.pack('<IBBBB', 0, 6, 8, 192, 4) + struct.pack('B', 2)
    return frame.pack()

def build_manual_control(x=0, y=0, z=0, r=0, target=1):
    frame = MAVLinkV2Frame()
    frame.msgid = 69
    frame.len = 26
    frame.sysid = 255
    frame.compid = 1
    frame.seq = _next_seq()
    payload = struct.pack('<hhhh', x, y, z, r)
    payload += struct.pack('<H', 0) + struct.pack('B', target)
    payload += struct.pack('<H', 0) + struct.pack('B', 0x80)
    payload += struct.pack('<hh', 0, 0)
    payload += struct.pack('<hhhh', 32767, 32767, 32767, 32767)
    frame.payload = payload
    return frame.pack()

def build_heartbeat_gcs():
    return build_heartbeat()

def build_command_long(cmd, param1=0, param2=0, param3=0, param4=0,
                      param5=0, param6=0, param7=0, target_sys=1, target_comp=1):
    """构建 COMMAND_LONG (ID=76) 帧
    cmd: MAV_CMD 命令码
    param1~7: 命令参数
    target_sys: 目标系统ID
    target_comp: 目标组件ID
    """
    frame = MAVLinkV2Frame()
    frame.msgid = 76
    frame.len = 33
    frame.sysid = 255
    frame.compid = 1  # GCS
    frame.seq = _next_seq()
    # MAVLink标准COMMAND_LONG载荷顺序: param1-7 + command + target_sys + target_comp + confirmation
    payload = struct.pack('<f', param1)           # param1
    payload += struct.pack('<f', param2)           # param2
    payload += struct.pack('<f', param3)           # param3
    payload += struct.pack('<f', param4)           # param4
    payload += struct.pack('<f', param5)           # param5
    payload += struct.pack('<f', param6)           # param6
    payload += struct.pack('<f', param7)           # param7
    payload += struct.pack('<H', cmd)              # command (MAV_CMD)
    payload += struct.pack('<B', target_sys)       # target_system
    payload += struct.pack('<B', target_comp)      # target_component
    payload += struct.pack('<B', 0)                # confirmation
    frame.payload = payload
    return frame.pack()

def build_arm_command(arm=True, target_sys=1, target_comp=1):
    """构建解锁/锁定指令，使用 MAVLink V1 帧封装发送
    arm=True: 解锁 (MAV_CMD_COMPONENT_ARM_DISARM, param1=1)
    arm=False: 锁定 (MAV_CMD_COMPONENT_ARM_DISARM, param1=0)

    解锁流程（需配合主循环连续发送）：
      1. 先发送 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(246) 重启通信
      2. 连续发送 ARM_DISARM(400) 指令直到收到 COMMAND_ACK
    """
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    frame = MAVLinkV2Frame()
    frame.msgid = 76
    frame.len = 33
    frame.sysid = 255   # 地面站
    frame.compid = 1    # GCS
    frame.seq = _next_seq()
    # MAVLink标准COMMAND_LONG载荷顺序: param1-7 + command + target_sys + target_comp + confirmation
    payload = struct.pack('<f', 1.0 if arm else 0.0)  # param1: 1=ARM, 0=DISARM
    payload += struct.pack('<f', 0.0)             # param2
    payload += struct.pack('<f', 0.0)             # param3
    payload += struct.pack('<f', 0.0)             # param4
    payload += struct.pack('<f', 0.0)             # param5
    payload += struct.pack('<f', 0.0)             # param6
    payload += struct.pack('<f', 0.0)             # param7
    payload += struct.pack('<H', MAV_CMD_COMPONENT_ARM_DISARM)  # command
    payload += struct.pack('<B', target_sys)       # target_system
    payload += struct.pack('<B', target_comp)      # target_component
    payload += struct.pack('<B', 0)                # confirmation
    frame.payload = payload
    # 使用 V1 帧格式发送，兼容性更好，不截断尾部零字节
    return frame.pack_v1()

def build_takeoff_command(altitude=10.0, pitch=15.0, target_sys=1, target_comp=1):
    """构建起飞指令，使用 MAVLink V1 帧封装发送
    MAV_CMD_NAV_TAKEOFF (cmd=22)
    param1: 俯仰角(度), 起飞时的最小俯仰角
    param2: 空(0)
    param3: 空(0)
    param4: 偏航角(度), NaN=当前朝向
    param5: 纬度(0=当前位置)
    param6: 经度(0=当前位置)
    param7: 起飞高度(米)
    """
    MAV_CMD_NAV_TAKEOFF = 22
    frame = MAVLinkV2Frame()
    frame.msgid = 76
    frame.len = 33
    frame.sysid = 255   # 地面站
    frame.compid = 1    # GCS
    frame.seq = _next_seq()
    # MAVLink标准COMMAND_LONG载荷顺序: param1-7 + command + target_sys + target_comp + confirmation
    payload = struct.pack('<f', pitch)              # param1: 俯仰角
    payload += struct.pack('<f', 0.0)               # param2: 空
    payload += struct.pack('<f', 0.0)               # param3: 空
    payload += struct.pack('<f', float('nan'))      # param4: 偏航角(NaN=当前朝向)
    payload += struct.pack('<f', 0.0)               # param5: 纬度(0=当前)
    payload += struct.pack('<f', 0.0)               # param6: 经度(0=当前)
    payload += struct.pack('<f', altitude)           # param7: 起飞高度(米)
    payload += struct.pack('<H', MAV_CMD_NAV_TAKEOFF)  # command
    payload += struct.pack('<B', target_sys)         # target_system
    payload += struct.pack('<B', target_comp)        # target_component
    payload += struct.pack('<B', 0)                  # confirmation
    frame.payload = payload
    return frame.pack_v1()

def build_land_command(target_sys=1, target_comp=1):
    """构建降落指令，使用 MAVLink V1 帧封装发送
    MAV_CMD_NAV_LAND (cmd=21)
    param1: 中止降落标志(0=正常降落)
    param2: 空(0)
    param3: 空(0)
    param4: 偏航角(度), NaN=当前朝向
    param5: 纬度(0=当前位置)
    param6: 经度(0=当前位置)
    param7: 着陆高度(0=当前地面高度)
    """
    MAV_CMD_NAV_LAND = 21
    frame = MAVLinkV2Frame()
    frame.msgid = 76
    frame.len = 33
    frame.sysid = 255   # 地面站
    frame.compid = 1    # GCS
    frame.seq = _next_seq()
    # MAVLink标准COMMAND_LONG载荷顺序: param1-7 + command + target_sys + target_comp + confirmation
    payload = struct.pack('<f', 0.0)                # param1: 正常降落
    payload += struct.pack('<f', 0.0)               # param2: 空
    payload += struct.pack('<f', 0.0)               # param3: 空
    payload += struct.pack('<f', float('nan'))      # param4: 偏航角(NaN=当前朝向)
    payload += struct.pack('<f', 0.0)               # param5: 纬度(0=当前)
    payload += struct.pack('<f', 0.0)               # param6: 经度(0=当前)
    payload += struct.pack('<f', 0.0)               # param7: 着陆高度
    payload += struct.pack('<H', MAV_CMD_NAV_LAND)  # command
    payload += struct.pack('<B', target_sys)         # target_system
    payload += struct.pack('<B', target_comp)        # target_component
    payload += struct.pack('<B', 0)                  # confirmation
    frame.payload = payload
    return frame.pack_v1()


# ============================================================
# 5. 飞行模式 & 姿态快照辅助
# ============================================================

MAIN_MODE_NAMES = {
    0:'手动', 1:'环绕', 2:'定高', 3:'定点', 4:'自动', 5:'特技', 6:'Offboard', 7:'增稳',
}
AUTO_SUB = {2:'起飞', 3:'跟踪', 4:'任务', 5:'返航', 6:'降落'}
LOITER_SUB = {0:'普通定点', 2:'定点避障'}

def decode_flight_mode(custom_mode):
    b = custom_mode.to_bytes(4,'little')
    main, sub = b[2], b[3]
    name = MAIN_MODE_NAMES.get(main, str(main))
    if main ==4 and sub in AUTO_SUB: name += f'({AUTO_SUB[sub]})'
    elif main ==3 and sub in LOITER_SUB: name += f'({LOITER_SUB[sub]})'
    return name

LAND_STATE = {0:'未知', 1:'地面', 2:'飞行中', 3:'降落中'}

def make_snapshot():
    """生成当前飞控数据快照文本"""
    lines = []
    if 'roll' in _cache_att:
        lines.append(f"  横滚={_cache_att['roll']:+7.2f}  俯仰={_cache_att['pitch']:+7.2f}  偏航={_cache_att['yaw']:+7.2f}")
    if 'groundspeed' in _cache_vfr:
        lines.append(f"  地速={_cache_vfr['groundspeed']:.2f}m/s  空速={_cache_vfr['airspeed']:.2f}m/s  油门={_cache_vfr['throttle']}%")
    if 'lat' in _cache_pos:
        lines.append(f"  位置=({_cache_pos['lat']:.6f}, {_cache_pos['lon']:.6f})  航向={_cache_pos['heading']}")
    if 'rel_alt' in _cache_pos:
        lines.append(f"  相对高={_cache_pos['rel_alt']:.1f}m  海拔={_cache_pos['alt']:.1f}m")
    if 'throttle' in _cache_vfr and _cache_vfr['climb'] is not None:
        lines.append(f"  上升率={_cache_vfr['climb']:.2f}m/s  高度={_cache_vfr['alt']:.1f}m")
    return '\n'.join(lines)

# 缓存
_cache_att = {}
_cache_pos = {}
_cache_vfr = {}
_cache_mode = ''
_cache_armed = False


# ============================================================
# 6. 键盘手动控制
# ============================================================

KEY_SPEED = 500
KEY_YAW_SPEED = 300
KEY_THROTTLE = 500

def get_keyboard_input(current_x, current_y, current_z, current_r):
    x, y, z, r = current_x, current_y, current_z, current_r
    quit_flag = False
    arm_cmd = None  # 'arm', 'disarm', 'takeoff', 'land'
    while msvcrt.kbhit():
        key = msvcrt.getch()
        if key == b'\x1b':
            quit_flag = True; break
        elif key == b' ':
            x, y, z, r = 0, 0, 0, 0
        elif key in (b'\x00', b'\xe0'):
            arrow = msvcrt.getch()
            if arrow == b'H': x = KEY_SPEED
            elif arrow == b'P': x = -KEY_SPEED
            elif arrow == b'K': y = -KEY_SPEED
            elif arrow == b'M': y = KEY_SPEED
        else:
            char = key.decode('ascii', errors='ignore').lower()
            if char == 'w': x = KEY_SPEED
            elif char == 's': x = -KEY_SPEED
            elif char == 'q': r = -KEY_YAW_SPEED
            elif char == 'e': r = KEY_YAW_SPEED
            elif char == 'r': z = KEY_THROTTLE
            elif char == 'f': z = -KEY_THROTTLE
            elif char == 'a': arm_cmd = 'arm'       # A键: 解锁
            elif char == 'd': arm_cmd = 'disarm'     # D键: 锁定
            elif char == 't': arm_cmd = 'takeoff'    # T键: 起飞
            elif char == 'l': arm_cmd = 'land'       # L键: 降落
            elif char == ' ': x, y, z, r = 0, 0, 0, 0
    return x, y, z, r, quit_flag, arm_cmd


# ============================================================
# 7. 主程序
# ============================================================

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未发现可用串口")
        return []
    print("\n可用串口列表:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} - {p.description}")
    return ports


def main():
    print("=" * 60)
    print("  飞控MAVLink串口通信 Demo (完整解析 + 键盘控制)")
    print("=" * 60)
    print("""
  操控按键:
    W/S前/后飞    Q/E左/右转    R/F上升/下降
    A键解锁(ARM)  D键锁定(DISARM)  Space急停  ESC退出
    T键起飞(TAKEOFF, 10m)  L键降落(LAND)
  观察功能:
    按 G 键 → 手动拍摄快照(执行前)
    松开按键后自动对比显示变化
""")

    ports = list_serial_ports()
    if not ports:
        input("\n按回车退出..."); return
    c = input(f"\n选择串口 [0-{len(ports)-1}] (默认0): ").strip() or "0"
    try: sel = ports[int(c)]
    except: sel = ports[0]
    bd = input("波特率 (默认57600): ").strip() or "57600"

    try:
        ser = serial.Serial(sel.device, int(bd), timeout=0.05)
        print(f"\n串口 {sel.device} 已打开 ({bd} baud)")
    except Exception as e:
        print(f"打开串口失败: {e}"); input(); return

    start_time = time.time()
    last_hb_send = 0
    last_manual_send = 0
    hb_ok = False

    joy_x, joy_y, joy_z, joy_r = 0, 0, 0, 0
    rx_buffer = b""
    MAX_BUF = 4096           # 缓冲区上限，超过则丢弃老数据
    last_debug = 0
    bytes_total = 0

    # 快照状态
    snapshot_before = None     # 保存指令前的快照
    snapshot_triggered = 0     # 触发时刻
    SHOW_DELAY = 1.5           # 指令发出后等待秒数再做对比

    # 上一次操控值（检测变化用）
    prev_joy = (0, 0, 0, 0)

    global _cache_att, _cache_pos, _cache_vfr, _cache_mode, _cache_armed
    _cache_att = {}
    _cache_pos = {}
    _cache_vfr = {}
    _print_count = {}  # 消息打印节流计数器

    print("\n等待飞控数据... (收到后自动开始)")
    print("=" * 60)

    try:
        while True:
            now = time.time()
            elapsed = now - start_time

            if now - last_hb_send >= 1.0:
                hb_pkt = build_heartbeat_gcs()
                ser.write(hb_pkt)
                # 只在首次发送或状态变化时打印
                if not hasattr(main, '_hb_printed') or not main._hb_printed:
                    hex_str = ' '.join(f'{b:02x}' for b in hb_pkt)
                    print(f"[发送] HEARTBEAT(GCS→飞控): {hex_str}")
                    if not hb_ok:
                        print("[状态] 等待飞控心跳回复中...")
                    main._hb_printed = True
                last_hb_send = now

            # ---- 键盘 ----
            if now - last_manual_send >= 0.1:
                joy_x, joy_y, joy_z, joy_r, quit, arm_cmd = get_keyboard_input(
                    joy_x, joy_y, joy_z, joy_r)
                if quit: break

                # 解锁/锁定/起飞/降落指令（连续发送3次确保飞控收到）
                if arm_cmd:
                    # 根据指令类型构建对应帧
                    if arm_cmd == 'arm':
                        label = "解锁(ARM)"
                        pkt = build_arm_command(arm=True)
                    elif arm_cmd == 'disarm':
                        label = "锁定(DISARM)"
                        pkt = build_arm_command(arm=False)
                    elif arm_cmd == 'takeoff':
                        label = "起飞(TAKEOFF)"
                        pkt = build_takeoff_command(altitude=10.0)
                    elif arm_cmd == 'land':
                        label = "降落(LAND)"
                        pkt = build_land_command()
                    else:
                        label = arm_cmd
                        pkt = None
                    if pkt:
                        if not hb_ok:
                            print(f"\n[警告] 未收到飞控心跳，但仍尝试发送{label}指令")
                        hex_str = ' '.join(f'{b:02x}' for b in pkt)
                        print(f"\n[发送] {label} 指令 x3 (MAVLink V1 COMMAND_LONG)")
                        print(f"  帧数据({len(pkt)}字节): {hex_str}")
                        # 逐字段解读
                        if pkt[0] == 0xFE:
                            plen = pkt[1]; seq = pkt[2]; sid = pkt[3]; cid = pkt[4]; mid = pkt[5]
                            pay = pkt[6:6+plen]
                            print(f"  STX=FE(V1) LEN={plen} SEQ={seq} SYS={sid} COMP={cid} MSG={mid}(0x{mid:02x})")
                            if mid == 76 and plen == 33:
                                import struct as _st
                                # 标准COMMAND_LONG顺序: param1-7 + command + target_sys + target_comp + confirmation
                                p1,p2,p3,p4,p5,p6,p7 = _st.unpack_from('<fffffff', pay, 0)
                                cmd, t_sys, t_comp, conf = _st.unpack_from('<HBBB', pay, 28)
                                CMD_NAMES = {400:'ARM_DISARM', 22:'TAKEOFF', 21:'LAND'}
                                cmd_name = CMD_NAMES.get(cmd, str(cmd))
                                print(f"  target_sys={t_sys} target_comp={t_comp} cmd={cmd}({cmd_name}) confirmation={conf}")
                                print(f"  param1={p1:.1f} param2={p2:.1f} param3={p3:.1f} param4={p4:.1f}")
                                print(f"  param5={p5:.1f} param6={p6:.1f} param7={p7:.1f}")
                            crc = pkt[6+plen]*256 + pkt[6+plen+1]
                            print(f"  CRC=0x{crc:04x}")
                        sys.stdout.flush()
                        for _ in range(3):
                            ser.write(pkt)
                            time.sleep(0.05)
                    # 发送前快照
                    snapshot_before = make_snapshot()
                    snapshot_triggered = now
                    print("─── [指令执行前] ───")
                    print(snapshot_before if snapshot_before else "  (暂无数据)")

                for vn in ['joy_x','joy_y','joy_z','joy_r']:
                    v = eval(vn)
                    if abs(v) > 50:
                        v = int(v*0.85)
                        if abs(v) < 50:
                            v = 0
                        if vn == 'joy_x': joy_x = v
                        elif vn == 'joy_y': joy_y = v
                        elif vn == 'joy_z': joy_z = v
                        elif vn == 'joy_r': joy_r = v

                if hb_ok:
                    mc_pkt = build_manual_control(joy_x, joy_y, joy_z, joy_r)
                    cur_joy = (joy_x, joy_y, joy_z, joy_r)
                    if cur_joy != (0,0,0,0):
                        hex_str = ' '.join(f'{b:02x}' for b in mc_pkt)
                        print(f"[发送] MANUAL_CONTROL x={joy_x} y={joy_y} z={joy_z} r={joy_r}: {hex_str}")
                    ser.write(mc_pkt)

                # 检测操控变化：从无到有 → 触发生成"执行前"快照
                cur_joy = (joy_x, joy_y, joy_z, joy_r)
                if cur_joy != (0,0,0,0) and prev_joy == (0,0,0,0):
                    snapshot_before = make_snapshot()
                    snapshot_triggered = now
                    print("\n─── [指令执行前] ───")
                    print(snapshot_before if snapshot_before else "  (暂无数据)")
                prev_joy = cur_joy
                last_manual_send = now

            # ---- 读取串口 ----
            try:
                if ser.in_waiting > 0:
                    d = ser.read(ser.in_waiting)
                    rx_buffer += d
                    bytes_total += len(d)
                    # 缓冲区过大时丢弃前半部分
                    if len(rx_buffer) > MAX_BUF:
                        rx_buffer = rx_buffer[-MAX_BUF:]
            except: pass

            # ---- 调试 ----
            if not hb_ok:
                # 检测数据特征
                has_fe = b'\xfe' in rx_buffer[:100]
                has_fd = b'\xfd' in rx_buffer[:100]
                has_mavlink = has_fe or has_fd

                if now - last_debug >= 3.0:
                    last_debug = now
                    if bytes_total == 0:
                        print(f"[调试] 运行{int(elapsed)}秒，未收到数据。检查接线/波特率({bd})")
                    elif not has_mavlink:
                        h = ' '.join(f'{b:02x}' for b in rx_buffer[:min(24, len(rx_buffer))])
                        print(f"[调试] 收到{bytes_total}字节，但未发现MAVLink帧头(0xFE/0xFD)")
                        print(f"[调试] hex: {h}...")
                        print(f"[调试] 可能是波特率不正确！请尝试: 57600 / 9600 / 38400")
                    elif has_fe and not has_fd:
                        print(f"[调试] 收到{bytes_total}字节，数据以V1(0xFE)为主，已启用V1解析")
                    else:
                        h = ' '.join(f'{b:02x}' for b in rx_buffer[:min(32, len(rx_buffer))])
                        print(f"[调试] 收到{bytes_total}字节 hex:{h}...")

            # ---- 完整解析帧 ----
            while rx_buffer:
                frame, rx_buffer = MAVLinkV2Frame.parse(rx_buffer)
                if frame is None: break

                # 使用 mavlink_parser 的 decode_frame 获取详细解析
                detail = mavlink_parser.decode_frame(frame)

                # 更新缓存（V2协议会截断末尾零字节，需补回标准长度）
                _PAYLOAD_LEN = {0:9, 30:28, 33:28, 74:20}
                _plen = _PAYLOAD_LEN.get(frame.msgid, len(frame.payload))
                _payload = frame.payload
                if len(_payload) < _plen:
                    _payload = _payload + b'\x00' * (_plen - len(_payload))

                if frame.msgid == 30:  # ATTITUDE
                    r, p, y = struct.unpack('<fff', _payload[4:16])
                    _cache_att['roll'] = r*180/3.14159
                    _cache_att['pitch'] = p*180/3.14159
                    _cache_att['yaw'] = y*180/3.14159
                elif frame.msgid == 33:  # GLOBAL_POSITION_INT
                    lat, lon, alt, rel = struct.unpack('<iiii', _payload[4:20])
                    hdg = struct.unpack('<H', _payload[26:28])[0]
                    vx, vy = struct.unpack('<hh', _payload[20:24])
                    _cache_pos['lat'] = lat/1e7; _cache_pos['lon'] = lon/1e7
                    _cache_pos['alt'] = alt/1000; _cache_pos['rel_alt'] = rel/1000
                    _cache_pos['heading'] = hdg
                elif frame.msgid == 74:  # VFR_HUD
                    asp, gsp, alt, climb, hd, thr = struct.unpack('<ffffhH', _payload[:20])
                    _cache_vfr['airspeed'] = asp; _cache_vfr['groundspeed'] = gsp
                    _cache_vfr['alt'] = alt; _cache_vfr['climb'] = climb
                    _cache_vfr['throttle'] = thr; _cache_vfr['heading'] = hd
                elif frame.msgid == 0:  # HEARTBEAT
                    if not hb_ok:
                        print("[状态] ✓ 已收到飞控心跳，连接正常！可以发送指令")
                    hb_ok = True
                    cm = struct.unpack('<I', _payload[:4])[0]
                    base = _payload[6]
                    new_mode = decode_flight_mode(cm)
                    new_armed = (base & 0x80) != 0
                    if new_mode != _cache_mode or new_armed != _cache_armed:
                        _cache_mode = new_mode
                        _cache_armed = new_armed
                        print(f"[心跳] 模式={new_mode}  状态={'解锁' if new_armed else '锁定'}")

                # 打印完整解析（首次心跳后开始，每类消息节流打印）
                if hb_ok:
                    # 节流：同类型消息每10帧只打印1次
                    _print_count[frame.msgid] = _print_count.get(frame.msgid, 0) + 1
                    if _print_count[frame.msgid] % 10 == 1:
                        # 提取关键行显示（简洁版）
                        lines = detail.split('\n')
                        key_lines = [l for l in lines if '=' in l and not l.strip().startswith('=') and not l.strip().startswith('─')]
                        for kl in key_lines[:6]:
                            if kl.strip():
                                print(f"  {kl.strip()}")

            # ---- 执行后对比快照 ----
            if snapshot_before and (elapsed - (snapshot_triggered - start_time)) >= SHOW_DELAY:
                snapshot_after = make_snapshot()
                if snapshot_after and snapshot_after != snapshot_before:
                    print(f"\n─── [指令执行后 {SHOW_DELAY:.0f}秒后] ───")
                    print(snapshot_after)
                    print("─── [变化对比] ───")
                    bef = snapshot_before.split('\n')
                    aft = snapshot_after.split('\n')
                    for b, a in zip(bef, aft):
                        if b != a:
                            print(f"  之前: {b}")
                            print(f"  之后: {a}")
                    print()
                snapshot_before = None

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n中断退出")
    finally:
        ser.close()
        print("串口已关闭")


if __name__ == "__main__":
    main()
