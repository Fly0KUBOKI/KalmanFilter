#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
シリアルセンサーデータ取得 - 設定ファイル
"""

# シリアルポート設定
SERIAL_PORT = 'COM3'
BAUDRATE = 921600
TIMEOUT = 1  # 秒

# ファイル出力設定
OUTPUT_DIRECTORY = '.'  # 現在のディレクトリ
OUTPUT_FILENAME_PREFIX = 'sensor_data'  # 例: sensor_data_20260115_103045.csv

# ロギング設定
VERBOSE = True  # 詳細出力を有効にするか
PROGRESS_INTERVAL = 100  # N行ごとに進捗を表示
ERROR_DISPLAY_LIMIT = 5  # 表示するエラーメッセージの最大数

# CSVカラム定義
CSV_COLUMNS = [
    'Timestamp',
    'Accel_X', 'Accel_Y', 'Accel_Z',      # 加速度 [m/s²]
    'Gyro_X', 'Gyro_Y', 'Gyro_Z',          # 角速度 [deg/s]
    'Mag_X', 'Mag_Y', 'Mag_Z',             # 磁場 [μT]
    'Pressure'                              # 気圧 [Pa]
]

# データパース設定
REGEX_PATTERN = r'([+-]?\d+\.\d+)\s+([+-]?\d+\.\d+)\s+([+-]?\d+\.\d+),([+-]?\d+\.\d+)\s+([+-]?\d+\.\d+)\s+([+-]?\d+\.\d+),([+-]?\d+\.\d+)\s+([+-]?\d+\.\d+)\s+([+-]?\d+\.\d+),([\d.]+)'

# 予期される値の範囲（オプション: 外れ値検出用）
EXPECTED_RANGES = {
    'Accel_X': (-20, 20),      # m/s²
    'Accel_Y': (-20, 20),
    'Accel_Z': (-20, 20),
    'Gyro_X': (-500, 500),     # deg/s
    'Gyro_Y': (-500, 500),
    'Gyro_Z': (-500, 500),
    'Mag_X': (-100, 100),      # μT
    'Mag_Y': (-100, 100),
    'Mag_Z': (-100, 100),
    'Pressure': (80000, 120000) # Pa
}
