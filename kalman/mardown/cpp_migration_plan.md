# C++ 化移行計画 — 短期実行版

**最終更新**: 2025年12月21日  
**ステータス**: PHASE 4 完了 → PHASE 5（浮動小数点精度改善）準備

---

## 📌 現在のタスク

### PHASE 4 結果（根本原因特定済）
✅ **浮動小数点精度差**: C++ `float32` vs MATLAB `float64`
- 影響: GPS 速度更新で ~1-2% の誤差
- 解決策: C++ の型を `float64` に変更

### 短期ロードマップ（PHASE 5）

| 優先度 | Phase | タスク | 所要時間 | 状態 |
|--------|-------|--------|---------|------|
| 🔴 高 | 5.1 | C++ `float32` → `float64` 変更 | 1-2h | ⏳ TODO |
| 🔴 高 | 5.2 | `build_mex()` でリビルド | 0.5h | ⏳ TODO |
| 🔴 高 | 5.3 | `run_batch_10sets()` 検証 | 1h | ⏳ TODO |
| 🟡 中 | 6.1 | Phase 3 API インターフェース確認 | 2h | ⏳ TODO |
| 🟡 中 | 6.2 | 単体テスト追加 (`tests/`) | 3-4h | ⏳ TODO |

---

## 🔧 実装手順（PHASE 5.1）

### 1. C++ 型変更

**ファイル**: `kalman/cpp/MEUKF/meukf_types.hpp`

```cpp
// 変更前:
typedef Eigen::Matrix<float, 15, 15> Matrix15x15;

// 変更後:
typedef Eigen::Matrix<double, 15, 15> Matrix15x15;
```

**対象ファイル一覧**:
- `kalman/cpp/MEUKF/meukf_types.hpp` — すべての `float` → `double`
- `kalman/cpp/ESKF/*.hpp` — 同様に型変更
- `kalman/cpp/MEUKF/*.cpp` — ポインタ型やキャスト確認

### 2. ビルド

```matlab
cd kalman/cpp/build
build_mex('mex_meukf_step')   % 単体ビルド
clear mex                      % MATLAB キャッシュクリア
```

### 3. テスト

```matlab
run_batch_10sets()             % 10セット検証
% 期待: GPS 速度更新誤差 < 1e-10 m/s
```

---

## 📊 完了した Phase

### Phase 0-2: C++化（✅ 完了）
| 関数/クラス | 状態 | テスト |
|------------|------|--------|
| `alpha_beta_step` | ✅ MEX化 | 10/10 PASS |
| `ema_update` | ✅ MEX化 | 10/10 PASS |
| `hampel_causal` | ✅ MEX化 | 10/10 PASS |
| `BiquadFilter` | ✅ MEX化 | 10/10 PASS |
| `AccelFilter` | ✅ MEX化 | 10/10 PASS |
| `SensorGyroFilter` | ✅ 廃止 | 10/10 PASS |

### Phase 3: 準備中
| 関数/クラス | 状態 | 次ステップ |
|------------|------|----------|
| `NoiseEstimator` | C++ 実装確認 | API 突合・テスト |
| `DivergenceGuard` | C++ 実装確認 | API 突合・テスト |

### Phase 4: 根本原因特定（✅ 完了）
| 分析項目 | 結果 |
|---------|------|
| GPS Kalman Gain | float32 vs float64 の差異確認 |
| 精度差 | ~1-2% 相対誤差 |
| 推奨解決策 | `float64` への型変更 |

---

## 🏗️ 依存関係（参考）

```
レイヤー 0（基本関数）
├── alpha_beta_step ✅
├── ema_update ✅
└── hampel_causal ✅

レイヤー 1（基本クラス）
├── BiquadFilter ✅
└── AccelFilter ✅

レイヤー 2（ユーティリティ）
├── NoiseEstimator ⏳
└── DivergenceGuard ⏳

レイヤー 3-6（センサーフィルタ・統合）
└── 後続 Phase
```

---

## ⚠️ 重要な注意

1. **MATLAB コマンド順序**
   ```matlab
   clear mex                    % 必須: 古い MEX キャッシュをクリア
   build_mex(...)              % ビルド
   run_batch_10sets()          % テスト
   ```

2. **ビルドエラー時**
   - ログ確認: `kalman/cpp/build/build_log.txt`
   - コンパイラ設定確認: `mex -setup C++`

3. **パリティ検証**
   - MEX と MATLAB の結果が `< 1e-10` で一致することを確認
   - GPS 速度更新が主要指標（Δv_x, Δv_y, Δv_z）

---

## 📚 参考リンク

- `PROJECT_STATUS.md` — 統合ステータス
- `CLEANUP_RECORD_2025_12_21.md` — 削除ファイル一覧
- `phase3_migration_notes.md` — Phase 3 実装ノート
- `kalman/cpp/build/build_mex.m` — ビルドスクリプト

---

**次ステップ**: PHASE 5.1 を実施（C++ 型変更 → ビルド → テスト）

