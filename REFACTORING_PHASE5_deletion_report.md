# REFACTORING_PHASE5_deletion_report

目的: Phase5 で削除／移動候補となるディレクトリ（`Lib/KF`, `Lib/EKF`, `Lib/UKF`, `Common/inc/*`, ルート `inc/`, `src/`）が現行コードでどこから参照されているかを完全に列挙する。

生成日: 2026-01-12

---

## 検索パターンと結果サマリ

- `Lib/KF` 関連 (#include "../../KF/inc/..." 等): 31 件
  - 代表的参照:
    - `kalman/cpp/Lib/ESKF/src/eskf_core.cpp` : #include "../../KF/inc/kalman_filter_core.hpp"
    - `kalman/cpp/Lib/ESKF/inc/eskf_helper.hpp` : #include "../../KF/inc/kf_operations.hpp"
    - `kalman/cpp/Lib/EKF/src/ekf_linear_update.cpp` : #include "../../KF/inc/kalman_filter_core.hpp"
    - `kalman/cpp/Lib/Common/inc/Validation/validation.hpp` : #include "../../../KF/inc/kf_operations.hpp"

- `Lib/EKF` 関連: 1 件（ドキュメント参照）
  - `kalman/cpp/markdown/LIB_FUNCTION_REFERENCE.md` に `EKF/inc/ekf_core.hpp` の見出しあり。
  - 実コード参照は `Lib/EKF/src/ekf_linear_update.cpp` 等で確認（EKF 自身のファイル群を参照）。

- `Lib/UKF` 関連: 7 件
  - `kalman/cpp/Lib/MEUKF/src/*` が UKF の `ukf_sigma_points.hpp`, `ukf_update.hpp` を include。

- `Common/inc/Math` 関連: 51 件（最も多い）
  - 主要参照先（抜粋）:
    - `kalman/cpp/Lib/ESKF/src/*`（`eskf_core.cpp`, `eskf_initializer.cpp`, `eskf_math.cpp`, `eskf_postprocess.cpp` 等）
    - `kalman/cpp/Lib/MEUKF/src/*`
    - `kalman/cpp/Lib/Quaternion/quaternion_functions.hpp`
    - `kalman/cpp/Lib/Matrix/fixed_matrix.hpp`
    - `kalman/cpp/MEX/Impl/mex_run_eskf_sensor_updates.hpp` / `mex_eskf_common.hpp`

- `Common/inc/Sensor` 関連: 10 件
  - 主要参照先（抜粋）:
    - `kalman/cpp/Lib/ESKF/inc/eskf_sensor_updates.hpp`
    - `kalman/cpp/Lib/ESKF/src/eskf_runner.cpp`
    - `kalman/cpp/MEX/Impl/mex_eskf_common.hpp`
    - `kalman/cpp/Lib/MEUKF/src/unified_filter.cpp`

- ルート `inc/`（`../inc/kalman_filter.hpp` 等）: 2 件
  - `kalman/cpp/src/kalman_filter.cpp`
  - `kalman/cpp/src/assemble_measurements.cpp`

---

## 完全参照リスト（ファイル:行）

以下は grep 結果の要約（ファイル:行）です。レビュー用に必要なら更に行単位で展開します。

- kalman/cpp/MEX/mex_meukf_step.cpp:2-3
- kalman/cpp/MEX/mex_eskf_initializer.cpp:2
- kalman/cpp/MEX/Impl/mex_eskf_initializer.hpp:6
- kalman/cpp/MEX/Impl/mex_run_eskf_impl.hpp:15,423
- kalman/cpp/MEX/Impl/mex_eskf_common.hpp:30-38,43-45
- kalman/cpp/MEX/Impl/mex_run_eskf_sensor_updates.hpp:15
- kalman/cpp/MEX/Impl/mex_type_conversion.hpp:3
- kalman/cpp/MEX/Impl/mex_helpers.hpp:3

- kalman/cpp/Lib/ESKF/src/eskf_core.cpp:6,9-11,14
- kalman/cpp/Lib/ESKF/src/eskf_runner.cpp:1-3,5,11
- kalman/cpp/Lib/ESKF/src/eskf_postprocess.cpp:1,3-4
- kalman/cpp/Lib/ESKF/src/eskf_initializer.cpp:1,4-9,14
- kalman/cpp/Lib/ESKF/src/eskf_sensor_updates.cpp:1-4
- kalman/cpp/Lib/ESKF/inc/eskf_helper.hpp:5-6,10
- kalman/cpp/Lib/ESKF/inc/eskf_math.hpp:8-10
- kalman/cpp/Lib/ESKF/inc/eskf_runner.hpp:10
- kalman/cpp/Lib/ESKF/inc/eskf_sensor_updates.hpp:8-9
- kalman/cpp/Lib/ESKF/inc/filter.hpp:3

- kalman/cpp/Lib/MEUKF/src/unified_filter.cpp:1-6,11
- kalman/cpp/Lib/MEUKF/src/meukf_update.cpp:1-5,7,13
- kalman/cpp/Lib/MEUKF/src/meukf_predict.cpp:1,3-5,12
- kalman/cpp/Lib/MEUKF/src/meukf_sigma_points.cpp:1,3
- kalman/cpp/Lib/MEUKF/inc/meukf_core.hpp:3
- kalman/cpp/Lib/MEUKF/inc/meukf_helpers.hpp:5
- kalman/cpp/Lib/MEUKF/inc/meukf_observation_models.hpp:8

- kalman/cpp/Lib/EKF/src/ekf_linear_update.cpp:1,5,7,9-11,13
- kalman/cpp/Lib/EKF/inc/ekf_core.hpp:8-10

- kalman/cpp/Lib/KF/inc/kalman_filter_core.hpp:7-10
- kalman/cpp/Lib/KF/inc/kf_operations.hpp: (referenced by multiple files listed above)

- kalman/cpp/Lib/UKF/inc/ukf_sigma_points.hpp:8
- kalman/cpp/Lib/UKF/inc/ukf_update.hpp: (referenced by MEUKF src)

- kalman/cpp/Lib/Matrix/fixed_matrix.hpp:4,7-8
- kalman/cpp/Lib/Quaternion/quaternion_functions.hpp:2,5-6

- kalman/cpp/Lib/Common/inc/filter_mgmt.hpp: (referenced by ESKF/ MEX)
- kalman/cpp/Lib/Common/inc/Math/* : many files across ESKF/MEUKF/Matrix/Quaternion/MEX
- kalman/cpp/Lib/Common/inc/Sensor/* : used by ESKF/MEUKF/MEX

- kalman/cpp/src/kalman_filter.cpp:8-9
- kalman/cpp/src/assemble_measurements.cpp:1

---

## 推奨レビュー方針（短く）

1. `Common/inc/Math` は依存が広範囲のため、最小単位（portable_math.hpp, math_utils.hpp, vector_utils.hpp など）を 1–2 個ずつ移動してビルド確認する（段階的移行）。
2. `Common/inc/Sensor` は ESKF/MEUKF/MEX が参照しているため、`Lib/Sensor` を早めに作成して include パスだけ先に切り替える手順が安全。
3. `Lib/KF`, `Lib/EKF`, `Lib/UKF` は相互参照があるため「完全削除」は慎重に。まずそれぞれのファイル単位で `Lib/Core` に移設可能か検討し、依存を差し替える。
4. `inc/` のルートファイルは `src/` から直接参照されているため、移動／削除前に `build_mex.m` と `src/` の include を修正する。

---

必要ならこのレポートを元に自動 include 置換スクリプトを作成します（PowerShell または sed）。どのファイル群を最初に移動しますか？
