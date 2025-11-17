# ArduPilot センサーフィルタリング - ドキュメント索引

**作成日**: 2025年11月17日  
**対象**: ArduPilot フィルタリングシステム（Gyro, Accel, Magnetometer, GPS, Barometer）

---

## 📄 ドキュメント一覧

### 1. **ARDUPILOT_SENSOR_FILTERING_ANALYSIS.md** ⭐ メイン分析
   - フィルタの種類（EMA、Biquad、Notch、Mode、Average、Derivative）
   - フィルタの構成と共通インターフェース
   - センサー別の実装（IMU、磁気計、気圧計、GPS）
   - 特殊な処理（外れ値検出、動的パラメータ、振動検出）
   - 実装の特徴（メモリ、計算量、マルチスレッド対応）
   - コードスニペット集

### 2. **ARDUPILOT_FILTER_TECHNICAL_REFERENCE.md** 🔧 技術リファレンス
   - Filter ライブラリの詳細仕様
   - 各フィルタの数学的背景
   - API仕様（ヘッダファイル、初期化、パラメータ）
   - 性能特性表
   - 計算最適化テクニック
   - デバッグ方法

### 3. **ARDUPILOT_FILTER_IMPLEMENTATION_GUIDE.md** 💻 実装ガイド
   - 目的別フィルタ選択ガイド
   - 実装パターン（6つの詳細例）
   - ベストプラクティス
   - デバッグのヒント
   - 完全なコード例

---

## 🔍 クイックレファレンス

### フィルタタイプ早見表

| フィルタ | CPU | メモリ | 遅延 | 外れ値対応 | 用途 |
|---------|-----|--------|------|---------|------|
| **EMA** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | 1サンプル | ❌ | 高速、低負荷 |
| **Biquad 2p** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | 2サンプル | ❌ | 標準ローパス |
| **Notch** | ⭐⭐⭐ | ⭐⭐⭐⭐ | 2サンプル | ❌ | 振動周波数抑圧 |
| **Mode** | ⭐⭐ | ⭐⭐ | 2.5サンプル | ✅ | 外れ値除去 |
| **Average** | ⭐⭐⭐ | ⭐⭐⭐ | 5サンプル | ❌ | 平滑化 |
| **Derivative** | ⭐⭐ | ⭐⭐ | 不定 | ❌ | 微分値計算 |

### センサー別フィルタ構成

#### 加速度計 (Accelerometer)
```
Raw Data → Biquad 20Hz (LPF) → EKF
      ↓
    (オプション)
  Harmonic Notch 250Hz
```

#### ジャイロ (Gyroscope)
```
Raw Data → Harmonic Notch (RPM同期) → Biquad 20Hz → EKF
```

#### 気圧計 (Barometer)
```
Raw Data → Mode Filter 5サンプル → Average 10サンプル → Altitude Calc
```

#### 磁気計 (Magnetometer)
```
Raw Data → LPF 10Hz → Compass Calibration → EKF
```

#### GPS
```
Raw Position → Outlier Check → EKF (内部Kalmanフィルタ)
```

---

## 📊 アーキテクチャ

### フィルタ階層構造
```
┌─────────────────────────────────────┐
│ Application Layer                    │ ← EKF, Controller
├─────────────────────────────────────┤
│ Filter Application Layer            │ ← IMU Processor, Sensor Fusion
├─────────────────────────────────────┤
│ Filter Library                      │
│ ├─ LowPassFilter (EMA)              │
│ ├─ LowPassFilter2p (Biquad)        │
│ ├─ NotchFilter                      │
│ ├─ ModeFilter (Median)              │
│ ├─ AverageFilter                    │
│ └─ DerivativeFilter                 │
├─────────────────────────────────────┤
│ HAL/Driver Layer                    │ ← SPI, I2C, Hardware Filter
├─────────────────────────────────────┤
│ Sensor Hardware                     │ ← IMU, Compass, GPS, Baro
└─────────────────────────────────────┘
```

### ソフトウェア処理フロー

```
Sensor Data
    ↓
┌─ Hardware Filter (optional)
    ↓
┌─ Outlier Detection
    ↓
┌─ Harmonic Notch (RPM sync)
    ↓
┌─ Software LPF (Biquad)
    ↓
┌─ EKF Input
    ↓
Navigation Filter (Kalman)
    ↓
Controller Output
```

---

## 🎯 使用例ナビゲーション

### パターン1: 標準的なQuadcopter
- **ドキュメント**: ARDUPILOT_FILTER_IMPLEMENTATION_GUIDE.md
- **セクション**: "2. IMU フィルタリング実装例"
- **コード**: `IMUSensorProcessor` クラス

### パターン2: 高振動環境（レーシングドローン）
- **ドキュメント**: ARDUPILOT_FILTER_IMPLEMENTATION_GUIDE.md
- **セクション**: "3. ノッチフィルタの動的周波数追従"
- **コード**: `AdaptiveHarmonicNotchFilter` クラス

### パターン3: GPS統合
- **ドキュメント**: ARDUPILOT_SENSOR_FILTERING_ANALYSIS.md
- **セクション**: "5. GPS フィルタ"
- **参考**: `AP_NavEKF3` (EKF内部フィルタ)

### パターン4: 気圧計（高度推定）
- **ドキュメント**: ARDUPILOT_FILTER_IMPLEMENTATION_GUIDE.md
- **セクション**: "5. 気圧計フィルタの実装"
- **コード**: `BarometerFilter` クラス

### パターン5: 外れ値対応
- **ドキュメント**: ARDUPILOT_FILTER_IMPLEMENTATION_GUIDE.md
- **セクション**: "4. 外れ値除去フィルタの実装"
- **コード**: `OutlierRejectionFilter` クラス

---

## 🔗 GitHub リソース

### 主要ライブラリ
- **Filter Library**: `libraries/Filter/`
  - `LowPassFilter.h/cpp` - EMAフィルタ
  - `LowPassFilter2p.h/cpp` - Biquadフィルタ
  - `NotchFilter.h/cpp` - ノッチフィルタ
  - `AverageFilter.h` - 平均フィルタ
  - `ModeFilter.h/cpp` - 中央値フィルタ
  - `DerivativeFilter.h/cpp` - 微分フィルタ

- **IMU Processing**: `libraries/AP_InertialSensor/`
  - `AP_InertialSensor.h` - IMUマネージャー
  - `AP_InertialSensor_Backend.cpp` - フィルタ適用エンジン
  - `AP_InertialSensor_Invensense.cpp` - Invensenseセンサー実装

- **Compass**: `libraries/AP_Compass/`
  - `AP_Compass_*.cpp` - 磁気計フィルタリング

- **Barometer**: `libraries/AP_Baro/`
  - `AP_Baro_*.cpp` - 気圧計フィルタリング

- **Navigation**: `libraries/AP_NavEKF2/`, `AP_NavEKF3/`
  - EKF内部フィルタ統合

### 例とテスト
- `libraries/Filter/examples/` - フィルタ使用例
- `Tools/FilterTestTool/` - フィルタテストツール（Python）

---

## 📈 パフォーマンス指標

### CPU使用量（典型的な値）

| 処理 | 周波数 | CPU % | 時間 (μs/sample) |
|------|--------|--------|------------------|
| 加速度計フィルタ | 1kHz | 0.5-1.0 | 0.84-1.68 |
| ジャイロフィルタ | 1kHz | 0.5-1.0 | 0.84-1.68 |
| ノッチフィルタ | 1kHz | 0.3-0.5 | 0.5-0.84 |
| 気圧計フィルタ | 50Hz | 0.01-0.02 | 0.3-0.6 |
| **合計** | **1kHz** | **1.5-2.5%** | **2.5-4.2** |

### メモリ使用量（バイト、Vector3f基準）

| フィルタタイプ | INS_MAX_INSTANCES=3 | 説明 |
|---|---|---|
| Biquad 2p | 192 | 3×(2遅延×Vector3f) |
| EMA | 144 | 3×(1状態×Vector3f) |
| Notch | 192 | 3×(2遅延×Vector3f) |
| Harmonic Notch | 1440 | 2フィルタ×3インスタンス×（上記） |
| **合計** | ~2KB | 典型的なシステム |

---

## 🛠️ トラブルシューティング

### 問題: フィルタがNaN/Infを出力
- **原因**: フィルタ係数の計算エラーまたはオーバーフロー
- **解決**: 
  - `LowPassFilter2p::reset()` を呼び出し
  - サンプリングレート ≥ 2×カットオフ周波数 確認
  - 参考: ARDUPILOT_FILTER_IMPLEMENTATION_GUIDE.md "7. デバッグのヒント"

### 問題: フィルタ遅延が大きい
- **原因**: カットオフ周波数が低すぎる
- **解決**:
  - カットオフ周波数を上げる（通常は20-30Hz）
  - Notchフィルタではなく単純なBiquadを使用
  - 参考: ARDUPILOT_FILTER_TECHNICAL_REFERENCE.md "性能特性"

### 問題: 振動が除去されない
- **原因**: Notchフィルタの中心周波数が不正確
- **解決**:
  - RPM値を確認（`calc_base_frequency(rpm)` の結果）
  - Nyquist定理確認（中心周波数 < サンプリング周波数×0.4）
  - 参考: ARDUPILOT_FILTER_IMPLEMENTATION_GUIDE.md "3. ノッチフィルタ"

---

## 📚 関連参考資料

### 数学的背景
1. **Butterworth フィルタ**
   - 最大平坦周波数応答
   - Q値 = 0.707（2次）

2. **Biquad フィルタ係数**
   - デジタル信号処理における標準実装
   - Direct Form II構造

3. **中央値フィルタ**
   - 外れ値に対するロバスト性
   - 非線形フィルタ

### 実装資料
- **ArduPilot Documentation**: https://ardupilot.org/
- **Filter Library Source**: https://github.com/ArduPilot/ardupilot/tree/master/libraries/Filter
- **DSP Tutorials**: 各フィルタの数学的導出

---

## 🚀 クイックスタート

### Step 1: 基本を理解する
```
読む順序:
1. ARDUPILOT_SENSOR_FILTERING_ANALYSIS.md (概要)
2. ARDUPILOT_FILTER_TECHNICAL_REFERENCE.md (詳細)
3. ARDUPILOT_FILTER_IMPLEMENTATION_GUIDE.md (実装)
```

### Step 2: 適切なフィルタを選択
```
質問:
- 外れ値対応が必要? → Mode Filter 使用
- 特定周波数を除去? → Notch Filter 使用
- 平滑化のみ? → Biquad 2次LPF 使用
```

### Step 3: 実装する
```cpp
// テンプレート
LowPassFilter2pVector3f filter;
filter.set_cutoff_frequency(sample_rate_hz, cutoff_hz);

for (int i = 0; i < num_samples; i++) {
    Vector3f raw = read_sensor();
    Vector3f filtered = filter.apply(raw);
}
```

### Step 4: テストする
```cpp
// 周波数応答確認
sweep_frequency(10, 200, 1);  // 10-200Hz

// 安定性確認
for (int i = 0; i < 100000; i++) {
    filter.apply(test_signal);
}

// NaN/Inf チェック
assert(!filtered.is_nan());
assert(!filtered.is_inf());
```

---

## 📝 ドキュメント構成

```
ArduPilot Sensor Filtering Documentation
├── ARDUPILOT_SENSOR_FILTERING_ANALYSIS.md        [概要・分析]
│   ├── 1. フィルタの種類 (6種類)
│   ├── 2. フィルタの構成 (階層、インターフェース)
│   ├── 3. センサー別実装 (IMU, 気圧, GPS等)
│   ├── 4. 特殊な処理 (外れ値, 振動, 適応化)
│   ├── 5. 実装の特徴 (メモリ, CPU, スレッド)
│   └── 6. コードスニペット (4つの実装例)
│
├── ARDUPILOT_FILTER_TECHNICAL_REFERENCE.md       [技術リファレンス]
│   ├── Filter ライブラリ構造 (ディレクトリ構成)
│   ├── 各フィルタ詳細仕様
│   │   ├── LowPassFilter (EMA)
│   │   ├── LowPassFilter2p (Biquad)
│   │   ├── NotchFilter
│   │   ├── ModeFilter
│   │   ├── AverageFilter
│   │   └── DerivativeFilter
│   ├── AP_InertialSensor での統合
│   ├── 計算最適化テクニック
│   ├── デバッグ方法
│   └── パフォーマンス指標
│
├── ARDUPILOT_FILTER_IMPLEMENTATION_GUIDE.md      [実装ガイド]
│   ├── 基本フロー (フィルタチェーン)
│   ├── 6つの実装例
│   │   1. IMU フィルタリング
│   │   2. 動的ノッチフィルタ
│   │   3. 外れ値除去
│   │   4. 気圧計フィルタ
│   │   5. 温度補正
│   │   6. 動的最適化
│   ├── パフォーマンスモニタリング
│   ├── ベストプラクティス
│   └── デバッグのヒント
│
└── INDEX.md (本ファイル)                          [索引・ナビゲーション]
    ├── ドキュメント一覧
    ├── クイックレファレンス
    ├── アーキテクチャ図
    ├── トラブルシューティング
    ├── クイックスタート
    └── リソースリンク
```

---

## ✅ チェックリスト

### フィルタ実装時
- [ ] サンプリングレート ≥ 2×カットオフ周波数
- [ ] NaN/Inf チェック実装
- [ ] エラーハンドリング（リセット機能）
- [ ] CPU負荷測定 (< 5% per フィルタ)
- [ ] メモリ使用量確認

### テスト時
- [ ] DCレスポンス確認 (-3dB点)
- [ ] 周波数スイープテスト
- [ ] ステップレスポンステスト
- [ ] 長時間安定性テスト (>1時間)
- [ ] 外れ値対応テスト

### デプロイ前
- [ ] 全フィルタのパラメータ文書化
- [ ] キャリブレーションデータ確認
- [ ] ログ記録機能有効化
- [ ] 異常検出メッセージ確認
- [ ] パフォーマンス指標ベースライン確立

---

## 🤝 サポート

### よくある質問

**Q: Biquad と EMA のどちらを選ぶべき?**  
A: CPU効率が必須 → EMA。より急峻なカットオフ → Biquad

**Q: ノッチフィルタの帯域幅は?**  
A: 通常20Hz。周波数が不安定な場合は広める（25-30Hz）

**Q: Notch + LPF を同時に使用すべき?**  
A: はい。Notchで特定周波数、LPFで全体的な平滑化

**Q: パフォーマンス測定方法は?**  
A: `uint32_t t0 = micros(); filter.apply(...); uint32_t dt = micros()-t0;`

---

**最終更新**: 2025年11月17日  
**ドキュメントバージョン**: 1.0  
**ArduPilot バージョン**: 4.5.x
