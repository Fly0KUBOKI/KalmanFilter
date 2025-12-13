````markdown
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

---

````}