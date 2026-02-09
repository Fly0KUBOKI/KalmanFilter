#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
シリアルポートからセンサーデータを取得してCSVに保存するプログラム
- COM3, ボーレート921600で接続
- 加速度（3値）、ジャイロ（3値）、磁気（3値）、気圧（1値）を取得
"""

import serial
import csv
import os
from datetime import datetime
import re
import sys


def parse_sensor_line(line: str):
    """
    センサーデータラインをパースする
    フォーマット: +ax +ay +az,+gx +gy +gz,+mx +my +mz,pressure
    
    Returns:
        tuple: (ax, ay, az, gx, gy, gz, mx, my, mz, pressure) or None if parsing fails
    """
    try:
        # 前後の空白を削除
        line = line.strip()

        # まずは "タイムスタンプのみ" の行にマッチするか確認
        ts_only = re.match(r'^([+-]?\d+(?:\.\d+)?)\s*$', line)
        if ts_only:
            return (ts_only.group(1),)

        # フォーマットパターンにマッチ
        # 受け取り例:
        # [TIMESTAMP ]ax ay az,+gx gy gz,+mx my mz,pressure
        # 先頭タイムスタンプは文字列のまま保持し、センサ値は float に変換する
        number = r'([+-]?\d+(?:\.\d+)?)'
        pattern = rf'^(?:{number}\s+)?{number}\s+{number}\s+{number},{number}\s+{number}\s+{number},{number}\s+{number}\s+{number},{number}$'

        match = re.match(pattern, line)
        if not match:
            return None

        groups = match.groups()
        # groups は先頭タイムスタンプを含めて11個、またはタイムスタンプ無しで10個（すべて文字列）
        if len(groups) == 11:
            raw_time = groups[0]
            sensor_strs = groups[1:]
        else:
            raw_time = None
            sensor_strs = groups

        try:
            sensor_vals = [float(g) for g in sensor_strs]
        except Exception:
            return None

        if raw_time is not None:
            return tuple([raw_time] + sensor_vals)
        else:
            return tuple(sensor_vals)
    except Exception as e:
        print(f"パース エラー: {e}")
        return None


def read_sensor_data(com_port='COM3', baudrate=921600, timeout=1, output_file=None):
    """
    シリアルポートからセンサーデータを読み込み、CSVに保存する
    
    Args:
        com_port: シリアルポート (デフォルト: 'COM3')
        baudrate: ボーレート (デフォルト: 921600)
        timeout: 読み込みタイムアウト (秒)
        output_file: 出力ファイルパス (Noneの場合は自動生成)
    """
    
    # 出力ファイル名を生成
    if output_file is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"sensor_data_{timestamp}.csv"
    
    # 出力ファイルの絶対パス
    if not os.path.isabs(output_file):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_file = os.path.join(script_dir, output_file)
    
    print(f"シリアルポート: {com_port}")
    print(f"ボーレート: {baudrate}")
    print(f"出力ファイル: {output_file}")
    print("=" * 60)
    print("センサーデータ取得開始... (Ctrl+C で停止)")
    print("=" * 60)
    
    # CSVファイル初期化: MATLAB シミュレーションの形式に合わせる
    # time, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, baro, gps_lat, gps_lon, gps_alt
    csv_header = ['time', 'accel_x', 'accel_y', 'accel_z',
                  'gyro_x', 'gyro_y', 'gyro_z',
                  'mag_x', 'mag_y', 'mag_z',
                  'baro', 'gps_lat', 'gps_lon', 'gps_alt']
    
    try:
        # シリアルポート接続
        ser = serial.Serial(
            port=com_port,
            baudrate=baudrate,
            timeout=timeout,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        
        if not ser.is_open:
            print(f"エラー: {com_port} を開くことができません")
            return False
        
        print(f"接続成功: {com_port}\n")
        
        # CSVファイルを開いて書き込み開始
        with open(output_file, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(csv_header)
            
            line_count = 0
            error_count = 0
            start_time = datetime.now()
            
            while True:
                try:
                    # シリアルポートからデータ読み込み
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8', errors='ignore')
                        
                        # センサーデータをパース
                        parsed = parse_sensor_line(line)
                        
                        if parsed is not None:
                            # parsed は以下のいずれか:
                            #  - (timestamp_str,)                     : タイムスタンプ単独行
                            #  - (ax,ay,az,gx,gy,gz,mx,my,mz,pressure) : センサ値のみ
                            #  - (timestamp_str, ax, ..., pressure)   : 先頭に送られた time を含む行
                            if len(parsed) == 1:
                                # タイムスタンプのみ行はCSVには書かず表示のみ
                                device_time_str = parsed[0]
                                print(device_time_str)
                                # skip writing sensor row
                                continue
                            else:
                                if isinstance(parsed[0], str):
                                    device_time_str = parsed[0]
                                    vals = parsed[1:]
                                else:
                                    device_time_str = None
                                    vals = parsed

                                ax, ay, az, gx, gy, gz, mx, my, mz, pressure = vals
                                # 時刻文字列があれば数値化して秒 (s) 単位に変換、なければ経過秒を使用
                                if device_time_str is not None:
                                    try:
                                        tnum = float(device_time_str)
                                        # デバイスは ms を送ることが多いため ms->s に変換
                                        time_s = tnum / 1000.0
                                    except Exception:
                                        # 解析できない場合は経過秒
                                        time_s = (datetime.now() - start_time).total_seconds()
                                else:
                                    time_s = (datetime.now() - start_time).total_seconds()

                                # コンソール表示
                                formatted_line = (
                                    f"{time_s:.2f} "
                                    f"{ax:+0.2f} {ay:+0.2f} {az:+0.2f},"
                                    f"{gx:+0.2f} {gy:+0.2f} {gz:+0.2f},"
                                    f"{mx:+0.2f} {my:+0.2f} {mz:+0.2f},"
                                    f"{pressure:.2f}"
                                )
                                print(formatted_line)

                                # CSV 出力 (MATLAB 形式): gps は未搭載なので 0 を出力
                                csv_row = [f"{time_s:.3f}",
                                           f"{ax:.7f}", f"{ay:.7f}", f"{az:.7f}",
                                           f"{gx:.7f}", f"{gy:.7f}", f"{gz:.7f}",
                                           f"{mx:.7f}", f"{my:.7f}", f"{mz:.7f}",
                                           f"{pressure:.6f}",
                                           "0", "0", "0"]
                                row = csv_row

                            writer.writerow(row)
                            csvfile.flush()  # 即座にディスクに書き込み

                            line_count += 1

                            # 進捗表示（100行ごと）
                            if line_count % 100 == 0:
                                print(f"取得済み: {line_count} 行")
                        else:
                            error_count += 1
                            if error_count <= 5:
                                print(f"警告: パース失敗: {line.strip()}")
                
                except KeyboardInterrupt:
                    print("\n\n停止コマンドを受け取りました...")
                    break
                except Exception as e:
                    print(f"エラー: {e}")
                    error_count += 1
        
        # 接続を閉じる
        ser.close()
        
        print("=" * 60)
        print(f"取得完了!")
        print(f"  総データ数: {line_count} 行")
        print(f"  パース失敗: {error_count} 行")
        print(f"  保存先: {output_file}")
        print("=" * 60)
        
        return True
    
    except serial.SerialException as e:
        print(f"シリアル接続エラー: {e}")
        print(f"ポートが存在し、利用可能か確認してください")
        return False
    except Exception as e:
        print(f"予期しないエラー: {e}")
        return False


def list_available_ports():
    """利用可能なシリアルポートを一覧表示"""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("利用可能なシリアルポートが見つかりません")
            return
        
        print("利用可能なシリアルポート:")
        for port, desc, hwid in ports:
            print(f"  {port}: {desc} ({hwid})")
    except Exception as e:
        print(f"ポート一覧取得エラー: {e}")


if __name__ == "__main__":
    # コマンドライン引数をチェック
    com_port = 'COM3'
    output_file = None
    
    if len(sys.argv) > 1:
        if sys.argv[1] in ['-h', '--help']:
            print("使用方法:")
            print(f"  python {sys.argv[0]} [COM_PORT] [OUTPUT_FILE]")
            print("\n例:")
            print(f"  python {sys.argv[0]}            (COM3のデフォルト設定)")
            print(f"  python {sys.argv[0]} COM4")
            print(f"  python {sys.argv[0]} COM3 mydata.csv")
            print("\n利用可能なポート:")
            list_available_ports()
            sys.exit(0)
        else:
            com_port = sys.argv[1]
    
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    
    # センサーデータ取得開始
    success = read_sensor_data(
        com_port=com_port,
        baudrate=921600,
        timeout=1,
        output_file=output_file
    )
    
    sys.exit(0 if success else 1)
