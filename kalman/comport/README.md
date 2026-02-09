# シリアルセンサーデータ取得ツール

このツールは、シリアルポート経由でセンサーデータを取得し、CSVファイルに保存します。

## 対応センサー

- **加速度計** (ICM-20948等): ax, ay, az
- **ジャイロスコープ**: gx, gy, gz  
- **磁気センサー** (BMM150等): mx, my, mz
- **気圧計** (BMP390等): pressure

## 必要なライブラリ

```bash
pip install pyserial
```

## 使用方法

### 基本的な使い方

```bash
python read_sensor_data.py
```

デフォルト設定:
- ポート: **COM3**
- ボーレート: **921600**
- 出力ファイル: `sensor_data_YYYYMMDD_HHMMSS.csv`

### カスタムポート指定

```bash
python read_sensor_data.py COM4
```

### 出力ファイルを指定

```bash
python read_sensor_data.py COM3 my_sensor_data.csv
```

### ヘルプ表示

```bash
python read_sensor_data.py -h
```

### 利用可能なポートを確認

```bash
python read_sensor_data.py -h
```

（出力内にポート一覧が表示されます）

## 出力CSVフォーマット

| Timestamp | Accel_X | Accel_Y | Accel_Z | Gyro_X | Gyro_Y | Gyro_Z | Mag_X | Mag_Y | Mag_Z | Pressure |
|-----------|---------|---------|---------|--------|--------|--------|-------|-------|-------|----------|
| 2026-01-15T10:30:45.123456 | 0.012 | -0.005 | 9.814 | 0.001 | -0.002 | 0.003 | 25.4 | -12.3 | 45.8 | 101325.0 |

### カラム説明

- **Timestamp**: ISO形式のタイムスタンプ（マイクロ秒精度）
- **Accel_X/Y/Z**: 加速度 [m/s²]
- **Gyro_X/Y/Z**: 角速度 [deg/s]
- **Mag_X/Y/Z**: 磁場 [μT]
- **Pressure**: 気圧 [Pa]

## データの見方

### Pythonで読み込み

```python
import pandas as pd

# CSVを読み込み
df = pd.read_csv('sensor_data_20260115_103045.csv')

# 最初の5行表示
print(df.head())

# 加速度の統計
print(df[['Accel_X', 'Accel_Y', 'Accel_Z']].describe())
```

### MATLABで読み込み

```matlab
% CSVを読み込み
T = readtable('sensor_data_20260115_103045.csv');

% 加速度のプロット
plot(T.Accel_X, T.Accel_Y, T.Accel_Z);
legend('X', 'Y', 'Z');
ylabel('Acceleration [m/s^2]');
```

## トラブルシューティング

### エラー: `SerialException: could not open port`

- ポート番号を確認: `python read_sensor_data.py -h`
- デバイスマネージャーで正しいCOMポートが割り当てられているか確認
- USBドライバをインストール（例：CP210x, CH340）

### エラー: `ModuleNotFoundError: No module named 'serial'`

```bash
pip install pyserial
```

### データが取得できない

1. シリアルモニター（Arduino IDE等）で直接データが来ているか確認
2. ボーレート（921600）が正しいか確認
3. デバイス側の送信フォーマットが`printf`の形式と一致しているか確認

## 開発者向け情報

### フォーマット仕様

デバイス側の送信フォーマット（C/C++）:

```c
printf("%+3.3f %+3.3f %+3.3f,%+3.3f %+3.3f %+3.3f,%+3.3f %+3.3f %+3.3f,%.2f\n",
       icm_cached.accel[0], icm_cached.accel[1], icm_cached.accel[2],
       icm_cached.gyro[0], icm_cached.gyro[1], icm_cached.gyro[2],
       bmm_cached.mag[0], bmm_cached.mag[1], bmm_cached.mag[2],
       dps_cached.pressure);
```

パース方式: 正規表現マッチング（`read_sensor_data.py` の `parse_sensor_line()` 参照）

### パース仕様

```
フォーマット: +ax +ay +az,+gx +gy +gz,+mx +my +mz,pressure
```

例:
```
+0.012 -0.005 +9.814,+0.001 -0.002 +0.003,+25.400 -12.300 +45.800,101325.45
```

## ライセンス

MITライセンス
