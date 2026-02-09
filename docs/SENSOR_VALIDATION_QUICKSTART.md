# センサーモデル検証 — クイックスタート

実際のセンサーデータとシミュレーションのセンサーモデルを比較して、物理モデルの一致を検証する手順です。

## 📋 準備

1. **ハードウェア**:
   - センサーデバイス（加速度・ジャイロ・磁気・気圧計搭載）
   - USB/シリアル接続ケーブル
   - （推奨）回転台または円軌道を描けるツール

2. **ソフトウェア**:
   - MATLAB R2020b以降
   - Python 3.x + pyserial
   - このリポジトリのクローン

---

## 🚀 基本的な検証フロー（5ステップ）

### ステップ1: 静止データで基本確認（5分）

#### 実センサーデータ取得
```bash
cd kalman/comport
python read_sensor_data.py COM3 real_stationary.csv
# → デバイスを水平面に静置して5分間記録
# → Ctrl+C で停止
```

#### シミュレーション生成
```matlab
cd kalman
generate_validation_sim('stationary', 300);  % 5分（300秒）静止
```

#### 比較
```matlab
compare_real_sim_sensors('comport/real_stationary.csv', 'GenerateData/sensor_data.csv');
```

**期待される結果**:
- ✅ 加速度Z軸平均: 実データ ≈ 9.81 m/s²（±5%）
- ✅ ジャイロ平均: 実データ ≈ 0 deg/s（バイアス確認）
- ✅ ノイズ標準偏差: 実データとシミュレーションが同程度

**もし不一致なら**:
- ❌ 加速度が9.81から大きくずれる → センサーキャリブレーション不足
- ❌ ジャイロが0から大きくずれる → バイアス補正が必要
- ❌ ノイズレベルが異なる → `config_params.m` のノイズパラメータ調整

---

### ステップ2: ヨー回転で角速度確認（30秒）

#### 実センサーデータ取得
```bash
python read_sensor_data.py COM3 real_yaw_rotation.csv
# → デバイスを水平に保ち、Z軸周りに右回りで1周（10秒）
# → 3周ほど繰り返し
# → Ctrl+C で停止
```

#### シミュレーション生成
```matlab
generate_validation_sim('yaw_rotation', 30);
```

#### 比較
```matlab
compare_real_sim_sensors('comport/real_yaw_rotation.csv', 'GenerateData/sensor_data.csv');
```

**期待される結果**:
- ✅ ジャイロZ軸平均: 実データ ≈ 36 deg/s（等速1周/10秒の場合）
- ✅ 磁気XY平面が円軌道を描く
- ✅ ジャイロZ軸を積分すると360°×回転数

**もし不一致なら**:
- ❌ ジャイロZ軸の符号が逆 → 座標系定義が異なる
- ❌ 磁気が円を描かない → ハードアイアン補正が必要

---

### ステップ3: 円運動で向心加速度確認（60秒）⭐ **最重要**

#### 実センサーデータ取得
```bash
python read_sensor_data.py COM3 real_circular.csv
# → デバイスを水平に保ち、半径0.5mの円軌道を右回りで等速移動
# → 周期10秒で3周以上
# → Ctrl+C で停止
```

**円運動の実施方法**:
- 回転台を使う（推奨）: 回転台の端（半径0.5m位置）にデバイスを固定
- 手で持つ場合: メジャーで半径0.5mの円を描き、10秒/周で移動

#### シミュレーション生成
```matlab
generate_validation_sim('circular', 60);
```

#### 比較
```matlab
compare_real_sim_sensors('comport/real_circular.csv', 'GenerateData/sensor_data.csv');
```

**期待される結果**:
- ✅ ジャイロZ軸: 実データ ≈ 36 deg/s
- ✅ 向心加速度: 実データ ≈ 0.197 m/s²（半径0.5m、周期10秒）
- ✅ 磁気XY平面が円軌道
- ✅ フィルタ推定位置が円軌道を描く

**理論値計算**:
```
角速度 ω = 360°/10s = 36 deg/s = 0.628 rad/s
向心加速度 a_c = ω²R = (0.628)² × 0.5 = 0.197 m/s²
```

**もし不一致なら**:
- ❌ 向心加速度が異なる → 加速度計のスケールファクター調整
- ❌ 軌跡が円でない → フィルタパラメータ（Q, R）調整
- ❌ 位置推定が発散 → 初期化またはセンサー統合ロジックの問題

---

### ステップ4: ロール/ピッチ回転で姿勢確認（30秒）

#### 実センサーデータ取得（ロール）
```bash
python read_sensor_data.py COM3 real_roll_rotation.csv
# → デバイスをX軸周りに±45°揺らす（周期5秒）
# → 6回往復
# → Ctrl+C で停止
```

#### シミュレーション生成
```matlab
generate_validation_sim('roll_rotation', 30);
```

#### 比較
```matlab
compare_real_sim_sensors('comport/real_roll_rotation.csv', 'GenerateData/sensor_data.csv');
```

**期待される結果**:
- ✅ ジャイロX軸が正弦波（振幅 ≈ ±18 deg/s）
- ✅ 加速度Y軸成分が重力の影響で変化（Y = g × sin(roll)）
- ✅ フィルタ推定Roll角が±45°に達する

---

### ステップ5: 統計レポート確認

各比較実行後、以下のファイルが生成されます:

```
Results/
├── sensor_comparison_report.csv       # 統計量サマリー
├── sensor_comparison_加速度計比較.png    # 加速度時系列・分布
├── sensor_comparison_ジャイロ比較.png    # ジャイロ時系列・分布
├── sensor_comparison_磁気計比較.png      # 磁気時系列・分布
└── sensor_comparison_パワースペクトル密度.png  # ノイズ特性
```

**レポートの見方**:
- `Mean_Diff`: 実データとシミュレーションの平均値差（小さいほど良い）
- `Std_Ratio`: ノイズ標準偏差の比（1.0に近いほど良い）
- `Correlation`: 時系列の相関係数（1.0に近いほど良い）

---

## 📊 推奨検証順序

| 順序 | 運動パターン | 目的 | 所要時間 | 難易度 |
|-----|------------|------|---------|-------|
| 1 | `stationary` | 重力・バイアス・ノイズレベル確認 | 5分 | ⭐ 簡単 |
| 2 | `yaw_rotation` | ジャイロZ軸・磁気計確認 | 30秒 | ⭐⭐ 普通 |
| 3 | `roll_rotation` | ジャイロX軸・重力分解確認 | 30秒 | ⭐⭐ 普通 |
| 4 | `pitch_rotation` | ジャイロY軸確認 | 30秒 | ⭐⭐ 普通 |
| 5 | `circular` | 向心加速度・総合確認 | 60秒 | ⭐⭐⭐ やや難 |
| 6 | `circular_osc` | 複合運動・フィルタ追従性 | 60秒 | ⭐⭐⭐⭐ 難 |

---

## 🔧 パラメータ調整

### ノイズパラメータ調整（`config_params.m`）

実データの標準偏差がシミュレーションと異なる場合:

```matlab
% kalman/GenerateData/config_params.m を編集
params.noise.accel_std = single(0.X);   % 実データのstdに合わせる
params.noise.gyro_std = single(0.X);    % 実データのstdに合わせる
params.noise.mag_std = single(X.X);     % 実データのstdに合わせる
params.noise.baro_std = single(X.X);    % 実データのstdに合わせる
```

調整後、再度シミュレーション生成して比較。

### センサーキャリブレーション

加速度計/磁気計のオフセット・スケールが不正確な場合、
実デバイス側でキャリブレーションを実施してください。

---

## 🎯 合格基準

### レベル1: 基本動作確認
- [ ] 静止時の加速度ノルムが 9.81 ± 0.5 m/s²
- [ ] 静止時のジャイロ平均が ±2 deg/s 以内
- [ ] ノイズ標準偏差比が 0.5 〜 2.0 の範囲

### レベル2: 物理モデル一致
- [ ] ヨー回転のジャイロZ積分が実測角度と ±5° 以内
- [ ] ロール/ピッチ回転の重力成分が理論値と ±10% 以内
- [ ] 磁気XY平面が円軌道（楕円率 < 0.2）

### レベル3: 高精度検証（オプション）
- [ ] 円運動の向心加速度が理論値と ±5% 以内
- [ ] フィルタ推定位置のRMS誤差が < 0.5m
- [ ] パワースペクトル密度の傾きが一致（ピンクノイズ: -10 dB/dec）

---

## 📚 詳細ドキュメント

- [SENSOR_MODEL_VALIDATION_PLAN.md](SENSOR_MODEL_VALIDATION_PLAN.md) — 詳細な検証計画
- [CPP_INPUT_OUTPUT_SPEC.md](CPP_INPUT_OUTPUT_SPEC.md) — センサーデータ仕様
- [copilot-instructions.md](../.github/copilot-instructions.md) — プロジェクト概要

---

## 🐛 トラブルシューティング

### Q: 実データのCSVが読み込めない
**A**: `read_sensor_data.py` の出力フォーマットがMATLAB simulation形式になっているか確認:
```csv
time, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, baro, gps_lat, gps_lon, gps_alt
```

### Q: ジャイロの符号が逆
**A**: デバイスの座標系定義が異なる可能性。シミュレーションの座標系は:
- 機体座標系: Forward(+X), Right(+Y), Down(+Z)
- 右手系、ZYX回転順序（Yaw→Pitch→Roll）

### Q: 磁気計が円を描かない
**A**: 
1. ハードアイアン/ソフトアイアン補正が必要
2. 周囲に磁気外乱源（磁石、金属）がないか確認
3. デバイスのキャリブレーション実施

### Q: 向心加速度が理論値と合わない
**A**:
1. 円運動の半径・周期を正確に測定し直す
2. デバイスの向き（接線方向 vs 中心向き）を確認
3. 加速度計のスケールファクターを調整

---

## 📞 サポート

問題が解決しない場合は、以下の情報を添えてissueを作成してください:

1. 運動パターン名
2. 実データCSVファイルの最初の10行
3. 比較レポート（`sensor_comparison_report.csv`）
4. エラーメッセージ（あれば）

---

**次のステップ**: まずは `stationary` テストから開始して、基本動作を確認しましょう！
