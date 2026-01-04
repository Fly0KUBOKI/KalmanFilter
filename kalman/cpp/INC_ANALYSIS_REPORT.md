# Inc ディレクトリ分析レポート

## 概要
`Inc` ディレクトリは**公開フォワーディングヘッダーレイヤー**として機能しており、`Lib` 内の実装を外部向けに公開しています。

## ファイル分類結果

### 1. フォワーディングヘッダー（安全 — 削除不可）
以下のファイルは、`Lib/*/inc` 内の対応ファイルへ単純にフォワード中。

```
Inc/matrix.hpp                                  → Lib/Matrix/fixed_matrix.hpp
Inc/quaternion.hpp                              → Lib/Quaternion/quaternion_functions.hpp
Inc/kalman_all.hpp                              → Lib/全体の集約ヘッダー
Inc/ESKF/eskf_core.hpp                          → Lib/ESKF/inc/eskf_core.hpp
Inc/ESKF/eskf_filter.hpp                        → Lib/ESKF/inc/eskf_filter.hpp
Inc/ESKF/eskf_helper.hpp                        → Lib/ESKF/inc/eskf_helper.hpp
Inc/ESKF/eskf_initializer.hpp                   → Lib/ESKF/inc/eskf_initializer.hpp
Inc/ESKF/eskf_math.hpp                          → Lib/ESKF/inc/eskf_math.hpp
Inc/ESKF/eskf_postprocess.hpp                   → Lib/ESKF/inc/eskf_postprocess.hpp
Inc/ESKF/eskf_runner.hpp                        → Lib/ESKF/inc/eskf_runner.hpp
Inc/ESKF/eskf_sensor_updates.hpp                → Lib/ESKF/inc/eskf_sensor_updates.hpp
Inc/ESKF/eskf_state.hpp                         → Lib/ESKF/inc/eskf_state.hpp
Inc/Common/filter_management.hpp                → Lib/Common/inc/filter_mgmt.hpp
Inc/Common/Sensor/sensor_filter.hpp             → Lib/Common/inc/Sensor/sensor_filter.hpp
Inc/Common/Sensor/sensor_preprocessor.hpp       → Lib/Common/inc/Sensor/sensor_preprocessor.hpp
Inc/Common/Validation/validation.hpp            → Lib/Common/inc/Validation/validation.hpp
Inc/Common/Math/math_utils.hpp                  → Lib/Common/inc/Math/math_utils.hpp
Inc/Common/Math/statistics.hpp                  → Lib/Common/inc/Math/statistics.hpp
Inc/Common/Math/vector_utils.hpp                → Lib/Common/inc/Math/vector_utils.hpp
Inc/MEUKF/meukf_core.hpp                        → Lib/MEUKF/inc/meukf_core.hpp
Inc/MEUKF/meukf_types.hpp                       → Lib/MEUKF/inc/meukf_types.hpp
Inc/MEUKF/unified_filter.hpp                    → Lib/MEUKF/inc/unified_filter.hpp
Inc/MEUKF/unified_types.hpp                     → Lib/MEUKF/inc/unified_types.hpp
```

**削除判定**: ❌ 削除禁止
- MEX が `MEX/Inc/mex_eskf_common.hpp` 経由で頻繁に使用
- 多くは 1–2 行のフォワーディングのみで無害
- `Lib` への依存を保つことで、整理された API レイヤーを維持

---

### 2. 要注意ファイル

#### 2.1 `Inc/kalman_filters.hpp`
**内容**: KF/UKF の統合ヘッダー、名前空間エイリアス定義
**問題**: 
- `#include "KF/kalman_filter_core.hpp"` を参照  → **対応ファイルが Inc に存在しない**（`.bak` のみ）
- 実装は `Lib/KF/inc/kalman_filter_core.hpp` にある

**判定**: ⚠️ 更新推奨
```
#include "KF/kalman_filter_core.hpp"     ← 間違い（Inc に存在しない）
                                         ↓ 修正案
#include "../Lib/KF/inc/kalman_filter_core.hpp"
```

---

### 3. バックアップファイル（既作成）
```
Inc/KF/kalman_filter_core.hpp.bak        （古い実装コピー）
Inc/KF/kf_core.hpp.bak                   （古い実装コピー）
Inc/UKF/ukf_sigma_points.hpp.bak         （Eigen依存、非推奨）
Inc/UKF/ukf_update.hpp.bak               （Eigen依存、非推奨）
```

**判定**: ✓ 保持推奨（参照用）

---

## 推奨アクション

| ファイル | 処置 | 理由 |
|---------|------|------|
| フォワーディングヘッダー（24ファイル） | **保持** | MEX で使用中、レイヤー構造に必須 |
| `kalman_filters.hpp` | **更新** | 参照先パスが不正（`KF/kalman_filter_core.hpp` → `../Lib/KF/inc/kalman_filter_core.hpp`） |
| `.bak` ファイル | **保持** | 参照履歴として有用 |

---

## 結論

**Inc は削除不可。フォワーディングレイヤーとして重要。**

ただし `kalman_filters.hpp` の参照パスを修正すること。

修正内容:
```cpp
// 修正前
#include "KF/kalman_filter_core.hpp"

// 修正後
#include "../Lib/KF/inc/kalman_filter_core.hpp"
```

