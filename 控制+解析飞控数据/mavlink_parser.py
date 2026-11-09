# -*- coding: utf-8 -*-
"""
飞控MAVLink数据帧完整解析工具
根据：地面站&飞控交互协议 V2.8.4
功能：解析串口收到的原始MAVLink V2二进制数据，逐字段解读

使用方式:
  1. 独立运行: python mavlink_parser.py
     输入十六进制字符串，回车解析
  2. 作为模块: from mavlink_parser import parse_frame
"""

import struct
import sys

# ============================================================
# 1. CRC-16/MCRF4XX
# ============================================================

MAVLINK_CRC_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
]


def crc_calculate(buf, crc=0xFFFF):
    for b in buf:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        crc = (crc >> 8) ^ MAVLINK_CRC_TABLE[tmp]
    return crc


# ============================================================
# 2. CRC_EXTRA 表
# ============================================================

MSG_CRC_EXTRA = {
    0: 50, 1: 124, 2: 217, 20: 214, 22: 220, 23: 168, 24: 24,
    26: 170, 30: 39, 32: 185, 33: 104, 34: 237, 35: 244,
    42: 28, 65: 118, 66: 148, 69: 243, 74: 20, 75: 158,
    76: 152, 77: 143, 84: 143, 86: 5, 124: 87,
    147: 154, 148: 178, 242: 36, 243: 85, 245: 130,
    253: 83, 330: 158, 500: 243, 602: 159,
}


# ============================================================
# 3. 帧解析
# ============================================================

STX_V2 = 0xFD
STX_V1 = 0xFE


class MavFrame:
    """解析后的 MAVLink 帧"""
    def __init__(self):
        self.magic = 0
        self.len = 0
        self.incompat = 0
        self.compat = 0
        self.seq = 0
        self.sysid = 0
        self.compid = 0
        self.msgid = 0
        self.payload = b""
        self.crc = 0
        self.raw = b""

    def __repr__(self):
        return f"[sys={self.sysid} comp={self.compid} msgid={self.msgid}(0x{self.msgid:x}) len={self.len}]"


def parse_frame(data):
    """
    从字节流中解析下一帧
    返回: (frame|None, remaining_bytes)
    """
    if len(data) < 12:
        return None, data

    # 查找帧头
    idx = -1
    for i, b in enumerate(data):
        if b == STX_V2:
            idx = i
            break
    if idx < 0:
        for i, b in enumerate(data):
            if b == STX_V1:
                return None, data[i+1:]
        return None, b""

    if len(data) < idx + 12:
        return None, data

    hdr = data[idx:idx+10]
    _, payload_len, incompat, compat, seq, sysid, compid = \
        struct.unpack('<BBBBBBB', hdr[:7])
    msgid = struct.unpack('<I', hdr[7:10] + b'\x00')[0]

    total_len = 10 + payload_len + 2
    if len(data) < idx + total_len:
        return None, data

    payload = data[idx+10:idx+10+payload_len]
    recv_crc = struct.unpack('<H', data[idx+10+payload_len:idx+10+payload_len+2])[0]

    # CRC 校验
    crc_extra = MSG_CRC_EXTRA.get(msgid, 0)
    c = crc_calculate(hdr[1:])
    c = crc_calculate(payload, c)
    c = crc_calculate(bytes([crc_extra]), c)

    if c != recv_crc:
        return None, data[idx+1:]

    f = MavFrame()
    f.magic = STX_V2
    f.len = payload_len
    f.incompat = incompat
    f.compat = compat
    f.seq = seq
    f.sysid = sysid
    f.compid = compid
    f.msgid = msgid
    f.payload = payload
    f.crc = recv_crc
    f.raw = data[idx:idx+total_len]
    return f, data[idx+total_len:]


# ============================================================
# 4. 各报文解字段解析器
# ============================================================

# --- 辅助函数 ---

def _to_hex(b):
    return ' '.join(f'{x:02x}' for x in b)


def _pad(payload, length):
    """V2协议会截断末尾零字节，补回标准长度"""
    if len(payload) < length:
        payload = payload + b'\x00' * (length - len(payload))
    return payload


def _rad2deg(r):
    return r * 180.0 / 3.1415926535


# 飞行模式
MAIN_MODES = {0:'手动', 1:'环绕', 2:'定高', 3:'定点', 4:'自动', 5:'特技', 6:'Offboard', 7:'增稳'}
AUTO_SUB = {2:'起飞', 3:'跟踪', 4:'任务', 5:'返航', 6:'降落'}
LOITER_SUB = {0:'普通定点', 2:'定点避障'}

def _decode_mode(cm):
    b = cm.to_bytes(4, 'little')
    main = b[2]
    sub = b[3]
    name = MAIN_MODES.get(main, f'{main}')
    if main == 4 and sub in AUTO_SUB:
        name += f'({AUTO_SUB[sub]})'
    elif main == 3 and sub in LOITER_SUB:
        name += f'({LOITER_SUB[sub]})'
    return name


GPS_FIX = {0:'无', 1:'无', 2:'2D', 3:'3D', 4:'DGPS', 5:'RTK浮点', 6:'RTK固定', 7:'静态'}
LAND_STATE = {0:'未知', 1:'地面', 2:'飞行中', 3:'降落中'}
SEVERITY = {0:'紧急', 1:'警报', 2:'严重', 3:'错误', 4:'警告', 5:'通知', 6:'信息', 7:'调试'}


# --- 具体解析器 ---

def parse_heartbeat(payload):
    """ID=0 HEARTBEAT (9字节)"""
    payload = _pad(payload, 9)
    cm, typ, aut, base, stat, ver = struct.unpack('<IBBBBB', payload[:9])
    cols = []
    cols.append(('custom_mode', f'{cm} ({_decode_mode(cm)})'))
    cols.append(('type', f'{typ}'))
    cols.append(('autopilot', f'{aut}'))
    cols.append(('base_mode', f'{base} ({"解锁" if base&0x80 else "锁定"})'))
    cols.append(('system_status', f'{stat}'))
    cols.append(('mavlink_version', f'{ver}'))
    return 'HEARTBEAT(0)', cols


def parse_sys_status(payload):
    """ID=1 SYS_STATUS (31字节)"""
    payload = _pad(payload, 31)
    (onboard_ctrl_sensors_present, onboard_ctrl_sensors_enabled,
     onboard_ctrl_sensors_health, load, voltage_battery,
     current_battery, battery_remaining, drop_rate_comm,
     errors_comm, errors_count1, errors_count2, errors_count3,
     errors_count4) = struct.unpack('<IIIHHhbHHHHHH', payload[:31])
    cols = []
    cols.append(('onboard_sensors_present', hex(onboard_ctrl_sensors_present)))
    cols.append(('onboard_sensors_enabled', hex(onboard_ctrl_sensors_enabled)))
    cols.append(('onboard_sensors_health', hex(onboard_ctrl_sensors_health)))
    cols.append(('load', f'{load/10:.1f}%'))
    cols.append(('voltage_battery', f'{voltage_battery/1000:.3f}V'))
    cols.append(('current_battery', f'{current_battery/100:.2f}A'))
    cols.append(('battery_remaining', f'{battery_remaining}%'))
    cols.append(('drop_rate_comm', f'{drop_rate_comm}'))
    cols.append(('errors_comm', f'{errors_comm}'))
    return 'SYS_STATUS(1)', cols


def parse_gps_raw_int(payload):
    """ID=24 GPS_RAW_INT (52字节) 北斗1原始位置，V2可能截断"""
    payload = _pad(payload, 30)
    lat = struct.unpack('<i', payload[8:12])[0]
    lon = struct.unpack('<i', payload[12:16])[0]
    alt = struct.unpack('<i', payload[16:20])[0]
    eph = struct.unpack('<H', payload[20:22])[0]
    epv = struct.unpack('<H', payload[22:24])[0]
    vel = struct.unpack('<H', payload[24:26])[0]
    cog = struct.unpack('<h', payload[26:28])[0]
    fix_type = payload[28]
    satellites = payload[29]
    cols = []
    cols.append(('纬度(lat)', f'{lat/1e7:.7f}'))
    cols.append(('经度(lon)', f'{lon/1e7:.7f}'))
    cols.append(('海拔(alt)', f'{alt/1000:.2f}m'))
    cols.append(('水平精度(eph)', f'{eph}厘米'))
    cols.append(('垂直精度(epv)', f'{epv}厘米'))
    cols.append(('地速(vel)', f'{vel/100:.2f}m/s'))
    cols.append(('定位类型(fix_type)', f'{fix_type} ({GPS_FIX.get(fix_type,"?")})'))
    cols.append(('卫星数(satellites)', f'{satellites}颗'))
    if len(payload) >= 52:
        payload = _pad(payload, 52)
        h_acc = struct.unpack('<I', payload[34:38])[0]
        v_acc = struct.unpack('<I', payload[38:42])[0]
        cols.append(('水平精度(h_acc)', f'{h_acc/1000:.3f}m'))
        cols.append(('垂直精度(v_acc)', f'{v_acc/1000:.3f}m'))
    return 'GPS_RAW_INT(24)', cols


def parse_gps2_raw(payload):
    """ID=124 GPS2_RAW (57字节) 北斗2原始位置，V2可能截断"""
    payload = _pad(payload, 35)
    lat = struct.unpack('<i', payload[8:12])[0]
    lon = struct.unpack('<i', payload[12:16])[0]
    alt = struct.unpack('<i', payload[16:20])[0]
    eph = struct.unpack('<H', payload[24:26])[0]
    epv = struct.unpack('<H', payload[26:28])[0]
    vel = struct.unpack('<H', payload[28:30])[0]
    fix_type = payload[32]
    satellites = payload[33]
    cols = []
    cols.append(('纬度(lat)', f'{lat/1e7:.7f}'))
    cols.append(('经度(lon)', f'{lon/1e7:.7f}'))
    cols.append(('海拔(alt)', f'{alt/1000:.2f}m'))
    cols.append(('水平精度(eph)', f'{eph}厘米'))
    cols.append(('垂直精度(epv)', f'{epv}厘米'))
    cols.append(('地速(vel)', f'{vel/100:.2f}m/s'))
    cols.append(('定位类型(fix_type)', f'{fix_type} ({GPS_FIX.get(fix_type,"?")})'))
    cols.append(('卫星数(satellites)', f'{satellites}颗'))
    return 'GPS2_RAW(124)', cols


def parse_attitude(payload):
    """ID=30 ATTITUDE (28字节)，V2可能截断末尾零"""
    payload = _pad(payload, 28)
    t, roll, pitch, yaw, rs, ps, ys = struct.unpack('<Iffffff', payload[:28])
    cols = []
    cols.append(('时间戳(time_boot_ms)', f'{t}ms'))
    cols.append(('横滚(roll)', f'{_rad2deg(roll):6.2f}度 ({roll:.4f}rad)'))
    cols.append(('俯仰(pitch)', f'{_rad2deg(pitch):6.2f}度 ({pitch:.4f}rad)'))
    cols.append(('偏航(yaw)', f'{_rad2deg(yaw):6.2f}度 ({yaw:.4f}rad)'))
    cols.append(('横滚角速度(rollspeed)', f'{rs:.4f} rad/s'))
    cols.append(('俯仰角速度(pitchspeed)', f'{ps:.4f} rad/s'))
    cols.append(('偏航角速度(yawspeed)', f'{ys:.4f} rad/s'))
    return 'ATTITUDE(30)', cols


def parse_global_position_int(payload):
    """ID=33 GLOBAL_POSITION_INT (28字节)，V2可能截断"""
    payload = _pad(payload, 28)
    t, lat, lon, alt, rel_alt, vx, vy, vz, hdg = \
        struct.unpack('<IiiiihhhH', payload[:28])
    cols = []
    cols.append(('时间戳(time_boot_ms)', f'{t}ms'))
    cols.append(('纬度(lat)', f'{lat/1e7:.7f}'))
    cols.append(('经度(lon)', f'{lon/1e7:.7f}'))
    cols.append(('海拔(alt)', f'{alt/1000:.2f}m'))
    cols.append(('相对高度(relative_alt)', f'{rel_alt/1000:.2f}m'))
    cols.append(('北向速度(vx)', f'{vx/100:.2f}m/s'))
    cols.append(('东向速度(vy)', f'{vy/100:.2f}m/s'))
    cols.append(('地向速度(vz)', f'{vz/100:.2f}m/s'))
    cols.append(('航向(hdg)', f'{hdg}度'))
    return 'GLOBAL_POSITION_INT(33)', cols


def parse_vfr_hud(payload):
    """ID=74 VFR_HUD (20字节)，V2可能截断"""
    payload = _pad(payload, 20)
    aspd, gspd, alt, climb, heading, throttle = \
        struct.unpack('<ffff h H', payload[:20])
    cols = []
    cols.append(('空速(airspeed)', f'{aspd:.2f}m/s'))
    cols.append(('地速(groundspeed)', f'{gspd:.2f}m/s'))
    cols.append(('海拔(alt)', f'{alt:.2f}m'))
    cols.append(('上升率(climb)', f'{climb:.2f}m/s'))
    cols.append(('航线(heading)', f'{heading}度'))
    cols.append(('油门(throttle)', f'{throttle}%'))
    return 'VFR_HUD(74)', cols


def parse_rc_channels(payload):
    """ID=65 RC_CHANNELS (42字节), V2可能截断"""
    payload = _pad(payload, 42)
    t = struct.unpack('<I', payload[0:4])[0]
    chs = list(struct.unpack('<HHHHHHHHHHHHHHHHHH', payload[4:40]))
    chancount = payload[40]
    rssi = payload[41]
    cols = []
    cols.append(('time_boot_ms', f'{t}ms'))
    for i in range(min(chancount, 18)):
        if chs[i] != 65535:
            cols.append((f'通道{i+1}(chan{i+1}_raw)', f'{chs[i]}'))
    cols.append(('有效通道数(chancount)', f'{chancount}'))
    cols.append(('信号强度(rssi)', f'{rssi}'))
    return 'RC_CHANNELS(65)', cols


def parse_rc_channels_scaled(payload):
    """ID=34 RC_CHANNELS_SCALED (22字节), V2可能截断"""
    payload = _pad(payload, 22)
    t, c1, c2, c3, c4, c5, c6, c7, c8, port, rssi = \
        struct.unpack('<IhhhhhhhhBB', payload[:22])
    cols = []
    cols.append(('time_boot_ms', f'{t}ms'))
    for i, v in enumerate([c1, c2, c3, c4, c5, c6, c7, c8], 1):
        if v != 32767:
            cols.append((f'校准通道{i}(chan{i})', f'{v}'))
    cols.append(('port', f'{port}'))
    cols.append(('rssi', f'{rssi}'))
    return 'RC_CHANNELS_SCALED(34)', cols


def parse_battery_status(payload):
    """ID=147 BATTERY_STATUS 旧电池报文"""
    payload = _pad(payload, 36)
    cc, ec, temp, *raw_voltages, cur_bat, bid, bfunc, btype, brem = \
        struct.unpack('<iih 20x h 10H x h BBBb', payload[:36])
    voltages = raw_voltages[:10]
    cols = []
    cols.append(('消耗电流(current_consumed)', f'{cc}mAh'))
    cols.append(('消耗能量(energy_consumed)', f'{ec}hJ'))
    cols.append(('温度(temperature)', f'{temp}°C'))
    if voltages[1] == 65535 and voltages[2] == 65535:
        voltage_v = struct.unpack('<I', bytes(voltages[4:8]))[0] / 1000.0
        cap = struct.unpack('<I', bytes(voltages[6:10]))[0]
        cols.append(('电池电压', f'{voltage_v:.2f}V'))
        cols.append(('电池电流', f'{cur_bat/100:.2f}A'))
    else:
        voltage_v = sum(v for v in voltages[:10] if v != 65535) / 1000.0
        cols.append(('电池电压(单节累加)', f'{voltage_v:.2f}V'))
    cols.append(('剩余电量(battery_remaining)', f'{brem}%'))
    return 'BATTERY_STATUS(147)', cols


def parse_battery_status_acfly(payload):
    """ID=602 BATTERY_STATUS_ACFLY 新电池报文"""
    payload = _pad(payload, 24)
    v, cap, seq, fault, temp, cycle, cur, bid, health, rem, cell_cnt = \
        struct.unpack('<III HhHh BBBB', payload[:24])
    cols = []
    cols.append(('电压(voltage)', f'{v/10:.1f}V' if v != 0xFFFFFFFF else '无效'))
    cols.append(('容量(capacity)', f'{cap}mAh' if cap != 0xFFFFFFFF else '无效'))
    cols.append(('温度(temperature)', f'{temp}°C' if temp != 0x7FFF else '无效'))
    cols.append(('循环次数(cycle_count)', f'{cycle}' if cycle != 0xFFFF else '无效'))
    cols.append(('电流(current)', f'{cur/10:.2f}A' if cur != 0x7FFF else '无效'))
    cols.append(('电池ID', f'{bid}'))
    cols.append(('健康度(health)', f'{health}%'))
    cols.append(('剩余百分比', f'{rem}%'))
    cols.append(('电池节数(cell_count)', f'{cell_cnt}' if cell_cnt != 0xFF else '无效'))
    return 'BATTERY_STATUS_ACFLY(602)', cols


def parse_extended_sys_state(payload):
    """ID=245 EXTENDED_SYS_STATE (2字节)"""
    payload = _pad(payload, 2)
    vtol = payload[0]
    landed = payload[1]
    cols = []
    cols.append(('vtol_state', f'{vtol}'))
    cols.append(('降落状态(landed_state)', f'{landed} ({LAND_STATE.get(landed,"?")})'))
    return 'EXTENDED_SYS_STATE(245)', cols


def parse_statustext(payload):
    """ID=253 STATUSTEXT 消息提示，V2可能截断末尾零"""
    sev = payload[0] if payload else 0
    try:
        text = payload[1:51].split(b'\x00')[0].decode('utf-8', errors='replace')
    except:
        text = '<解码失败>'
    cols = []
    cols.append(('严重程度(severity)', f'{sev} ({SEVERITY.get(sev,"?")})'))
    cols.append(('消息内容(Text)', text))
    if len(payload) >= 55:
        mid = struct.unpack('<H', payload[51:53])[0]
        cols.append(('id', f'{mid}'))
        cols.append(('chunk_seq', f'{payload[54]}'))
    return 'STATUSTEXT(253)', cols


def parse_system_time(payload):
    """ID=2 SYSTEM_TIME"""
    payload = _pad(payload, 12)
    t_unix = struct.unpack('<Q', payload[0:8])[0]
    t_boot = struct.unpack('<I', payload[8:12])[0]
    from datetime import datetime
    cols = []
    if t_unix > 1000000000000:
        dt = datetime.fromtimestamp(t_unix / 1000000)
        cols.append(('UNIX时间(time_unix_usec)', f'{dt.strftime("%Y-%m-%d %H:%M:%S.%f")}'))
    else:
        cols.append(('UNIX时间(time_unix_usec)', f'{t_unix}us'))
    cols.append(('启动时间(time_boot_ms)', f'{t_boot}ms'))
    return 'SYSTEM_TIME(2)', cols


def parse_mission_current(payload):
    """ID=42 MISSION_CURRENT"""
    payload = _pad(payload, 4)
    seq = struct.unpack('<H', payload[0:2])[0]
    cols = [('当前航点序号(seq)', f'{seq}')]
    if len(payload) >= 8:
        rseq = struct.unpack('<H', payload[2:4])[0]
        cols.append(('断点航点序号(resumeSeq)', f'{rseq}'))
    if len(payload) >= 38:
        payload = _pad(payload, 38)
        _, _, _, _, _, dist, trig_int, spd, rad = \
            struct.unpack('<fffffffff', payload[2:38])
        cols.append(('断点距离', f'{dist:.1f}m'))
        cols.append(('触发间距', f'{trig_int:.1f}m'))
        cols.append(('速度', f'{spd:.1f}m/s'))
        cols.append(('半径', f'{rad:.1f}m'))
    return 'MISSION_CURRENT(42)', cols


def parse_home_position(payload):
    """ID=242 HOME_POSITION (60字节)，V2可能截断"""
    payload = _pad(payload, 52)
    lat, lon, alt = struct.unpack('<iii', payload[0:12])
    cols = []
    cols.append(('返航点纬度(latitude)', f'{lat/1e7:.7f}'))
    cols.append(('返航点经度(longitude)', f'{lon/1e7:.7f}'))
    cols.append(('返航点海拔(altitude)', f'{alt/1000:.2f}m'))
    if len(payload) >= 60:
        payload = _pad(payload, 60)
        ax = struct.unpack('<f', payload[40:44])[0]
        ay = struct.unpack('<f', payload[44:48])[0]
        az = struct.unpack('<f', payload[48:52])[0]
        if ax != 0:
            a_lat = struct.unpack('<i', bytes(struct.pack('<f', ax)))[0]
            a_lon = struct.unpack('<i', bytes(struct.pack('<f', ay)))[0]
            cols.append(('备降点纬度(approach_x)', f'{a_lat * 1e-7:.7f}'))
            cols.append(('备降点经度(approach_y)', f'{a_lon * 1e-7:.7f}'))
            cols.append(('备降点海拔(approach_z)', f'{az/1000:.2f}m'))
    return 'HOME_POSITION(242)', cols


def parse_autopilot_version(payload):
    """ID=148 AUTOPILOT_VERSION (78字节)，V2可能截断"""
    payload = _pad(payload, 78)
    caps, _, fw_ver, _, _, board = struct.unpack('<QQIIII', payload[:36])
    a = (fw_ver >> 24) & 0xFF
    b = (fw_ver >> 16) & 0xFF
    c = (fw_ver >> 8) & 0xFF
    uid2 = payload[60:78]
    uid0 = struct.unpack('<I', uid2[0:4])[0]
    uid1 = struct.unpack('<I', uid2[4:8])[0]
    uid2_val = struct.unpack('<I', uid2[8:12])[0]
    uid_hex = f'{uid0:X}{uid1:X}{uid2_val:X}'
    board_names = {5: 'K9飞控'}
    cols = []
    cols.append(('固件版本(flight_sw_version)', f'{a}.{b}.{c}'))
    cols.append(('飞控类型(board_version)', f'{board} ({board_names.get(board,"?")})'))
    cols.append(('序列号(uid2)', uid_hex))
    return 'AUTOPILOT_VERSION(148)', cols


def parse_command_ack(payload):
    """ID=77 COMMAND_ACK (10字节)，V2可能截断
    标准格式: command(H2) + result(B1) + progress(B1) + result_param2(i4) + target_system(B1) + target_component(B1)
    """
    payload = _pad(payload, 10)
    cmd, result, progress, rp2 = struct.unpack('<HBBi', payload[:8])
    ts = payload[8]
    tc = payload[9]
    result_str = {0:'成功', 1:'失败', 2:'拒绝', 3:'进行中', 4:'不支持'}
    cols = []
    cols.append(('命令ID(command)', f'{cmd}'))
    cols.append(('执行结果(result)', f'{result} ({result_str.get(result,"?")})'))
    cols.append(('进度(progress)', f'{progress}'))
    cols.append(('result_param2', f'{rp2}'))
    cols.append(('target_system', f'{ts}'))
    cols.append(('target_component', f'{tc}'))
    return 'COMMAND_ACK(77)', cols


def parse_obstacle_distance(payload):
    """ID=330 OBSTACLE_DISTANCE 雷达距离"""
    payload = _pad(payload, 8)
    dists = list(struct.unpack('<72H', payload[8:152])) if len(payload) >= 152 else []
    cols = []
    if dists:
        forward = dists[0] if dists[0] != 65535 else '--'
        right = dists[4] if len(dists) > 4 and dists[4] != 65535 else '--'
        backward = dists[8] if len(dists) > 8 and dists[8] != 65535 else '--'
        left = dists[12] if len(dists) > 12 and dists[12] != 65535 else '--'
        down = dists[32] if len(dists) > 32 and dists[32] != 65535 else '--'
        cols.append(('前方距离', f'{forward}cm'))
        cols.append(('右方距离', f'{right}cm'))
        cols.append(('后方距离', f'{backward}cm'))
        cols.append(('左方距离', f'{left}cm'))
        cols.append(('下方(定高)距离', f'{down}cm'))
    return 'OBSTACLE_DISTANCE(330)', cols


# ============================================================
# 5. 解析器注册表
# ============================================================

MSG_PARSERS = {
    0:   parse_heartbeat,
    1:   parse_sys_status,
    2:   parse_system_time,
    24:  parse_gps_raw_int,
    30:  parse_attitude,
    33:  parse_global_position_int,
    34:  parse_rc_channels_scaled,
    42:  parse_mission_current,
    65:  parse_rc_channels,
    74:  parse_vfr_hud,
    77:  parse_command_ack,
    124: parse_gps2_raw,
    147: parse_battery_status,
    148: parse_autopilot_version,
    242: parse_home_position,
    245: parse_extended_sys_state,
    253: parse_statustext,
    330: parse_obstacle_distance,
    602: parse_battery_status_acfly,
}

MSG_NAMES = {
    0: 'HEARTBEAT', 1: 'SYS_STATUS', 2: 'SYSTEM_TIME',
    20: 'PARAM_REQUEST_READ', 22: 'PARAM_VALUE', 23: 'PARAM_SET',
    24: 'GPS_RAW_INT', 26: 'SCALED_IMU',
    30: 'ATTITUDE', 32: 'LOCAL_POSITION_NED', 33: 'GLOBAL_POSITION_INT',
    34: 'RC_CHANNELS_SCALED', 35: 'RC_CHANNELS_RAW',
    42: 'MISSION_CURRENT', 43: 'MISSION_REQUEST_LIST',
    44: 'MISSION_COUNT', 45: 'MISSION_CLEAR_ALL', 47: 'MISSION_ACK',
    51: 'MISSION_REQUEST_INT', 65: 'RC_CHANNELS',
    66: 'REQUEST_DATA_STREAM', 69: 'MANUAL_CONTROL',
    73: 'MISSION_ITEM_INT', 74: 'VFR_HUD',
    75: 'COMMAND_INT', 76: 'COMMAND_LONG', 77: 'COMMAND_ACK',
    84: 'SET_POSITION_TARGET_LOCAL_NED', 86: 'SET_POSITION_TARGET_GLOBAL_INT',
    124: 'GPS2_RAW', 147: 'BATTERY_STATUS',
    148: 'AUTOPILOT_VERSION', 242: 'HOME_POSITION', 243: 'SET_HOME_POSITION',
    245: 'EXTENDED_SYS_STATE', 253: 'STATUSTEXT',
    330: 'OBSTACLE_DISTANCE', 500: 'COMPONENT_EXTENSION43',
    602: 'BATTERY_STATUS_ACFLY',
}


# ============================================================
# 6. 单帧解析 + 格式化输出
# ============================================================

# 标准载荷长度表（V2协议会截断末尾零字节，需要补回）
EXPECTED_LEN = {
    0:9, 1:31, 2:12, 22:25, 24:52, 30:28, 33:28, 34:22,
    42:38, 65:42, 74:20, 77:10, 124:57, 147:54, 148:78,
    242:60, 245:2, 253:55, 330:167, 602:72,
}


def decode_frame(frame):
    """解析一帧 MAVLink 数据，返回可读字符串"""
    msg_name = MSG_NAMES.get(frame.msgid, f'UNKNOWN(0x{frame.msgid:x})')

    lines = []
    lines.append(f"{'='*60}")
    lines.append(f"  报文: {msg_name}  ID={frame.msgid}")
    lines.append(f"  系统: sysid={frame.sysid}  compid={frame.compid}  seq={frame.seq}")
    lines.append(f"  载荷: {frame.len} 字节")
    raw_hex = _to_hex(getattr(frame, 'raw', b'')[:12]) if hasattr(frame, 'raw') and frame.raw else '(无原始数据)'
    lines.append(f"  原始: {raw_hex}")
    lines.append(f"{'─'*60}")

    # V2协议截断末尾零字节，补回标准长度再解析
    payload = frame.payload
    expected = EXPECTED_LEN.get(frame.msgid, len(payload))
    if len(payload) < expected:
        payload = payload + b'\x00' * (expected - len(payload))

    parser = MSG_PARSERS.get(frame.msgid)
    if parser:
        result, fields = parser(payload)
        if result is None:
            lines.append(f"  [解析失败] {fields}")
        else:
            for name, value in fields:
                lines.append(f"  {name:30s} = {value}")
    else:
        lines.append(f"  (未实现解析器，原始载荷 {frame.len} 字节)")
        lines.append(f"  hex: {_to_hex(frame.payload)}")

    lines.append(f"{'='*60}")
    return '\n'.join(lines)


def parse_hex_string(hex_str):
    """解析十六进制字符串，返回所有可解析的帧"""
    hex_str = hex_str.replace(' ', '').replace('\n', '').replace('\r', '')
    try:
        data = bytes.fromhex(hex_str)
    except ValueError as e:
        return f"[错误] 十六进制格式无效: {e}"

    results = []
    while data:
        frame, data = parse_frame(data)
        if frame is None:
            break
        results.append(decode_frame(frame))
    return '\n\n'.join(results) if results else "未找到有效的 MAVLink V2 帧"


# ============================================================
# 7. 命令行交互
# ============================================================

def main():
    print("=" * 60)
    print("  飞控MAVLink数据帧解析工具")
    print("  协议版本: 地面站&飞控交互协议 V2.8.4")
    print("=" * 60)
    print("  支持解析的报文:")
    for mid in sorted(MSG_PARSERS.keys()):
        name = MSG_NAMES.get(mid, '')
        print(f"    ID={mid:3d}  {name}")
    print()
    print("  输入十六进制字符串(支持带空格/换行)")
    print("  例如: FD 09 00 00 01 01 01 00 00 00 ...")
    print("  输入空行退出")
    print("-" * 60)

    while True:
        try:
            line = input("\nhex > ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            break

        result = parse_hex_string(line)
        print(result)


if __name__ == '__main__':
    main()
