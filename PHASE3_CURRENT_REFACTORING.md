# Phase 3: 現在のリファクタリング計画

**更新日**: 2026年1月9日  
**ステータス**: 🚀 実装中  
**焦点**: `sensor_filter.hpp` 段階的分割 → 次に `meukf_core.cpp` 分割

---

## 📊 現状サマリー

### ✅ 完了した作業（2026-01-09）

| タスク | 詳細 | 状態 |
|--------|------|------|
| **OutlierDetector 抽出** | `outlier_detector.hpp` 新規作成、分割実装完了 | ✅ |
| **namespace 曖昧性解決** | グローバル修飾子（`::`）でコンパイル成功 | ✅ |
| **MEX ビルド** | 両 MEX targets (mex_run_eskf, mex_meukf_step_v2) | ✅ |
| **回帰テスト** | 10/10 PASS (batch_10sets_log_20260109_110320.txt) | ✅ |

### 📈 数値安定性確認
```
Position RMSE (overall): Mean=0.3136 m ✅
  X: 0.2081 m, Y: 0.2211 m, Z: 0.0761 m
Velocity RMSE: 0.5841 m/s ✅
Attitude RMSE: Roll=0.3133°, Pitch=0.3112°, Yaw=0.7890° ✅
```

---

## 🔄 フェーズ2.5: `sensor_filter.hpp` 分割（完了）

### 実装内容
```
sensor_filter.hpp (845行)
  ├─ ema_filter.hpp (新規, 抽出完了)
  ├─ biquad_filter.hpp (新規, 抽出完了)
  ├─ alpha_beta_filter.hpp (新規, 抽出完了)
  ├─ outlier_detector.hpp (新規, 抽出完了 ← 2026-01-09)
  ├─ robust_statistics.hpp (新規, 抽出完了)
  └─ sensor_filter.hpp (ディスパッチャー化, 150行に縮小)
```

### ✅ 実施済み
1. **EMAFilter分割**: `ema_filter.hpp` → 全機能移動
2. **BiquadLowpassFilter分割**: `biquad_filter.hpp` → 全機能移動
3. **AlphaBetaFilter分割**: `alpha_beta_filter.hpp` → 全機能移動
4. **NoiseEstimator・DivergenceGuard**: `robust_statistics.hpp` に移動
5. **OutlierDetector分割**: `outlier_detector.hpp` に移動 ← **本日完了**
6. **namespace曖昧性解決**: グローバル修飾子（`::`）で解決
7. **SensorFilterLib**: dispatcher化（全内部フィルタをinclude）

### 🔍 技術的改善点
- **Include順序**: namespace外でincludeして曖昧性を回避
- **型安全性**: `::common::math::cm`, `::kf::ops` でグローバル修飾
- **コード分割**: ファイルごとに単一責任（EMA, Biquad, Outlier検出など）

---

## 🎯 フェーズ3: `meukf_core.cpp` 分割（次のステップ）

### 現況
```
meukf_core.cpp: 1346行（超大型ファイル）
  ├─ predict_step() ~400行
  ├─ generate_sigma_points() ~350行
  ├─ update_step() ~400行
  └─ helper functions ~200行
```

### 分割計画
```
MEUKF/src/
├── meukf_core.cpp (dispatcher, ~200行)
├── meukf_predict.cpp (新規, ~400行)
├── meukf_sigma_points.cpp (新規, ~350行)
└── meukf_update.cpp (新規, ~400行)
```

### 実装予定
| ステップ | タスク | 期限 | 状態 |
|---------|--------|------|------|
| 3.1 | meukf_predict.cpp 抽出 | 1日 | ⏳ 予定中 |
| 3.2 | meukf_sigma_points.cpp 抽出 | 1日 | ⏳ 予定中 |
| 3.3 | meukf_update.cpp 抽出 | 1日 | ⏳ 予定中 |
| 3.4 | dispatcher化 & ビルド検証 | 1日 | ⏳ 予定中 |
| 3.5 | 回帰テスト & 精度確認 | 1日 | ⏳ 予定中 |

---

## ⚠️ Phase 3 実装時の注意（失敗から学んだ教訓）

### チェックリスト（必須実施）
- [ ] **分割前に依存関係を grep で全検索**
  ```bash
  grep -R "predict_step\|generate_sigma\|update_step" kalman/cpp/MEX --include="*.hpp"
  grep -R "predict_step\|generate_sigma\|update_step" kalman/cpp/Lib --include="*.hpp"
  ```
- [ ] **各モジュールで単独コンパイルテスト**
  ```bash
  # 概念的: 新しいヘッダーファイルが全必要なincludeを持つか確認
  ```
- [ ] **分割後、必ず短い動作確認**
  ```matlab
  clear mex; build_mex(); clear mex; run_simulation(42, true);
  ```
- [ ] **回帰テストで Max Innovation > 0 を確認**
  ```matlab
  run_batch_10sets();  % 10/10 PASS を確認
  ```
- [ ] **数値差 ±0.01m 以内を確認**

### 再発防止のポイント
1. **stub実装を完全に移植** — コピペではなく、論理検証してから移動
2. **分割後は短い実行テストが必須** — ビルド成功 ≠ 機能成功
3. **出力ログの確認** — Max Innovation, RMSE値をチェック
4. **段階的実装** — 1ファイルずつ分割→テスト→次へ

---

## 📋 次のアクション（優先順位）

### 🔴 HIGH（すぐに実施）
1. [ ] meukf_core.cpp の predict_step 抽出開始
2. [ ] 単体テスト実装（既存コードと数値比較）
3. [ ] ビルド & 短いシミュレーション確認

### 🟠 MEDIUM（1週間以内）
1. [ ] meukf_core.cpp 完全分割完了
2. [ ] Mahalanobis/Innovation 統一計画策定
3. [ ] 回帰テスト拡充（外れ値ケース 3種追加）

### 🟡 LOW（2週間以内）
1. [ ] ドキュメント最終更新
2. [ ] コミット & PR準備
3. [ ] UKF抽出計画の検討

---

## 📊 進捗トラッキング

### Week 1（1月6-10日）
- ✅ 2026-01-06～09: Phase 2完了 & sensor_filter.hpp分割
- ✅ 2026-01-09: OutlierDetector抽出完了 & 回帰10/10 PASS
- 🔄 2026-01-09: meukf_core.cpp分割開始予定

### Week 2（1月13-17日）
- ⏳ meukf_core.cpp 完全分割
- ⏳ 最終回帰テスト＆検証
- ⏳ ドキュメント完成

### 期待完了日
**2026年1月17日（金）** — Phase 3完全完了予定

