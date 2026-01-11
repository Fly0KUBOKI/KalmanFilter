# Phase4 ChangeLog

実施日: 2026-01-11

概要:
- 目的: 静的メソッドのみのクラスを名前空間関数へ移行し、可読性と一貫性を向上。
- アプローチ: 小バッチで呼び出し側を置換、各バッチで `build_mex()` と `run_batch_10sets()` による回帰検証を実施。

主要変更ファイル:
- kalman/cpp/Lib/Common/inc/Math/math_utils.hpp  (namespace 宣言・宣言追加)
- kalman/cpp/Lib/Common/src/math_utils_constants.cpp (`common::math::EPS`, `PI` 定義)
- kalman/cpp/Lib/Common/inc/filter_mgmt.hpp (`common::covariance`, `common::state` ラッパ追加)
- kalman/cpp/MEX/Impl/mex_run_eskf_sensor_updates.hpp (復元)
- その他: `kalman/cpp/Lib/ESKF/src/*`, `kalman/cpp/Lib/Quaternion/*`, `kalman/cpp/Lib/Common/src/Sensor/*` など多数（約33参照を段階的に更新）

ビルド / 回帰結果:
- ビルドログ: kalman/cpp/build/build_mex_log_20260111_233614.txt (両MEX成功)
- 回帰ログ: kalman/Results/log/batch_10sets_log_20260111_233723.txt (10/10 PASS)

今後の手順:
1. 変更をローカルでコミットし、ブランチを作成してリモートへプッシュ
2. PR を作成し、`docs/FAILURE_PREVENTION.md` とビルド／回帰ログを添付

推奨コミットメッセージ例:
```
Phase4: convert static-only classes to namespaces
- MathUtils -> common::math
- Add common::covariance and common::state wrappers
- Restore mex_run_eskf_sensor_updates.hpp
- Fix call sites and indexing issues
- Build and regression: 10/10 PASS (see Results/log)
```

---
