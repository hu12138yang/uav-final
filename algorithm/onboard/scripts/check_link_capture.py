#!/usr/bin/env python3
"""Validate real observation evidence; exit nonzero when navigation inputs are not ready."""
import csv
import math
from pathlib import Path
import sys


def check(directory):
    path = Path(directory)
    with (path / 'timing.csv').open(newline='') as source:
        rows = list(csv.DictReader(source))
    if len(rows) < 5:
        raise ValueError('至少需要 5 条约 1 Hz 的状态记录')
    end = float(rows[-1]['elapsed'])
    recent = [row for row in rows if end - float(row['elapsed']) <= 5]
    if len(recent) < 5 or float(recent[-1]['elapsed']) - float(recent[0]['elapsed']) < 3.5:
        raise ValueError('缺少连续稳定观察窗口')
    failures = []
    for field, threshold, label in [('gps_fix', 6, 'RTK 固定解'), ('clock_ok', 1, '飞控校时'),
                                    ('pose_fresh', 1, '位姿新鲜度'), ('ptp_ready', 1, '雷达 PTP')]:
        if not all(math.isfinite(float(row[field])) and float(row[field]) >= threshold for row in recent):
            failures.append(label)
    for field, label in [('lidar_packets', '雷达收包'), ('aligned_frames', '点云对齐'), ('map_frames', '真实点云入图')]:
        values = [float(row[field]) for row in recent]
        if not all(math.isfinite(value) for value in values) or any(b <= a for a, b in zip(values, values[1:])):
            failures.append(label + '未持续更新')
    errors = [float(row['crc_errors']) for row in recent]
    if not all(math.isfinite(value) for value in errors) or errors[-1] != errors[0]:
        failures.append('稳定窗口内 MAVLink CRC 错误增长')
    with (path / 'scans.csv').open(newline='') as source:
        if not any(row.get('aligned') == '1' and row.get('timing_ready') == '1' and
                   row.get('packet_policy_ok') == '1' for row in csv.DictReader(source)):
            failures.append('没有满足校时和报文策略的实测点云帧')
    if failures:
        raise ValueError('；'.join(failures))
    return '观察输入链路通过：RTK、飞控校时、位姿、PTP、点云对齐和地图持续更新。飞控控制执行及外部地面站接收仍需分别验收。'


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit('用法：python3 check_link_capture.py <record-dir>')
    try:
        print(check(sys.argv[1]))
    except (OSError, ValueError, KeyError) as error:
        sys.exit('输入链路未通过：' + str(error))
