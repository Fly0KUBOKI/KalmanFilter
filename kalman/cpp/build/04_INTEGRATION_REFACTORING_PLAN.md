# 統合・分離リファクタリング実装計画書（Phase-Based）

**策定日**: 2025-12-30  
**プロジェクト**: KalmanFilter MEX 統合・ソース分離  
**目標**: 複数の独立 MEX を統合し、ソースを 組織的に分離

---

## エグゼクティブ・サマリー

### ゴール

```
現状（phase6）:
  - 12個の独立 MEX ファイル（各々 1 つの機能）
  - ビルド・メンテナンスが複雑
  - MATLAB と C++ の境界が曖昧

目標状態（phase7 以降）:
  - 4個の統合 MEX（mex_meukf_step_v2, mex_sensor_filter, 
    mex_eskf_update_postprocess, mex_run_eskf）
  - ソースコード を Inc/ (ヘッダー), Src/ (実装) に分離
  - 一元管理，保守性向上
  - ビルド時間短縮，デバッグ容易
```

### 期間・リソース

| 項目 | 見積 |
|-----|------|
| **全体期間** | 5-7日 |
| **開発時間** | 40-50 時間 |
| **テスト時間** | 10-15 時間 |
| **リスク缓衝** | 20% |

---

## フェーズ詳細

### Phase 1: 準備と基礎構造の整備（1日）

**目的**: 統合前の基盤構築，ドキュメント・スクリプト整備

#### 1.1 ファイル構造の設計

**作業内容**:
- `Inc/ESKF/`, `Inc/Common/`, `Inc/MEX/` の階層設計
- `Src/ESKF/`, `Src/Common/` へソース移動
- `MEX/` ディレクトリ: 統合対象を整理

**成果物**:
```
Inc/ESKF/
├── eskf_core.hpp
├── eskf_runner.hpp
├── eskf_initializer.hpp
├── eskf_postprocess.hpp
└── eskf_state.hpp

Inc/Common/
├── Math/
│   ├── fixed_matrix.hpp
│   ├── vector_utils.hpp
│   └── quaternion_lib.hpp
├── Sensor/
│   ├── sensor_filter.hpp
│   └── sensor_preprocessor.hpp
└── filter_management.hpp

Src/ESKF/
├── eskf_core.cpp
├── eskf_runner.cpp
├── eskf_postprocess.cpp
└── ...

Src/Common/
└── ...
```

**チェックリスト**:
- [ ] ディレクトリ構造作成
- [ ] 既存ファイルの複製・配置
- [ ] インクルードガード確認

#### 1.2 ドキュメント・ツール準備

**作業内容**:
- `INCLUDE_DEPENDENCY.md`: インクルード依存グラフ
- `PROJECT_STRUCTURE.md`: プロジェクト構造
- `check_includes.m`: インクルード順序チェック
- `INTEGRATION_PHASES.md`: このファイル

**成果物**:
```
kalman/cpp/
├── INCLUDE_DEPENDENCY.md
├── PROJECT_STRUCTURE.md
├── CODE_REVIEW_CHECKLIST.md
└── build/
    ├── check_includes.m
    ├── verify_build.m
    └── INTEGRATION_PLAN.md
```

**チェックリスト**:
- [ ] 4 つのドキュメント作成
- [ ] 2 つのスクリプト作成
- [ ] README に計画へのリンク追加

#### 1.3 ビルドスクリプト拡張

**作業内容**:
- `build_mex.m`: テンプレートチェック機能を追加
- `verify_build.m`: ビルド後の自動テスト機能を追加
- `check_includes.m`: インクルード順序の自動チェック

**修正例**:
```matlab
% build_mex.m (修正)
function build_mex(targets, varargin)
    % 新規: テンプレート型チェック
    if ~check_template_types()
        error('Template type validation failed');
    end
    
    % 新規: インクルード順序チェック
    if ~check_includes('MEX/mex_run_eskf.cpp')
        error('Include order check failed');
    end
    
    % ... 既存のビルド処理 ...
end
```

**チェックリスト**:
- [ ] build_mex.m に type check 追加
- [ ] verify_build.m を実装
- [ ] check_includes.m を実装

#### 1.4 テスト環境セットアップ

**作業内容**:
- テスト用スクリプト: `verify_phase_X.m` テンプレート作成
- CI/CD 的なビルド検証の仕組み

**成果物**:
```
kalman/
├── verify_phase1.m  (各フェーズ用テストスクリプト)
├── verify_phase2.m
├── verify_phase3.m
├── verify_phase4.m
└── verify_phase5.m
```

**チェックリスト**:
- [ ] テストスクリプトテンプレート作成
- [ ] 基本的な run_simulation 統合

#### 1.5 Git フロー設定

**作業内容**:
- Feature ブランチの作成
- コミット戦略の文書化

**コマンド**:
```bash
git checkout -b feature/integrate-mex-phase1
git branch -M feature/integrate-mex-full  # 全統合用
```

**チェックリスト**:
- [ ] feature ブランチ作成
- [ ] main から分岐を確認

---

### Phase 2: 型・ユーティリティ層の基礎構築（1.5日）

**目的**: テンプレート型、ユーティリティ関数の分離・統合

#### 2.1 Type・Math ライブラリの分離（0.5日）

**作業内容**:
- `fixed_matrix.hpp`, `vector_utils.hpp`, `quaternion_lib.hpp` を `Inc/Common/Math/` に配置
- インクルード順序を最適化

**対象ファイル**:
```
移動元 → 移動先:
Lib/Matrix/fixed_matrix.hpp      → Inc/Common/Math/fixed_matrix.hpp
Lib/Matrix/vector_utils.hpp      → Inc/Common/Math/vector_utils.hpp
Lib/Quaternion/quaternion.hpp    → Inc/Common/Math/quaternion.hpp
Lib/Quaternion/quaternion_lib.hpp → Inc/Common/Math/quaternion_lib.hpp
```

**チェックリスト**:
- [ ] ファイル配置完了
- [ ] インクルードガード確認
- [ ] build_mex.m で参照パス更新

#### 2.2 mex_type_conversion の統合（0.5日）

**作業内容**:
- `MEX/mex_type_conversion.hpp` を `Inc/MEX/` に移動
- テンプレートの明示的指定を確認（matToVector<N>, matToMatrix<N, M>）

**チェックリスト**:
- [ ] ファイル移動完了
- [ ] テンプレートパラメータの使用を確認
- [ ] TEMPLATE_TEST.hpp でテスト

#### 2.3 Filter Management の統合（0.5日）

**作業内容**:
- `MEX/mex_filter_management.cpp` を `mex_run_eskf.cpp` へ統合
- フィルター管理ユーティリティ関数を `Inc/Common/filter_management.hpp` へ配置

**統合対象**:
```cpp
// mex_filter_management.cpp から以下を統合
- create_identity_P(...)     → Inc/Common/filter_management.hpp
- reset_bias_covariance(...) → Inc/Common/filter_management.hpp
```

**チェックリスト**:
- [ ] 関数を header へ移動
- [ ] mex_run_eskf.cpp から呼び出し確認
- [ ] `build_mex()` でエラーなし
- [ ] `verify_phase2.m` で動作確認

#### 2.4 ビルド・テスト検証（0時間）

**作業内容**: 上記が完了したら実施

```matlab
build_mex({'mex_meukf_step_v2', 'mex_sensor_filter'})
% 既存の 2 MEX がまだビルド成功するか確認
verify_phase2()
```

**コミット**:
```
commit: "refactor(phase2): Separate type/math/utility layer"
```

---

### Phase 3: ESKF コア層の分離（1日）

**目的**: ESKF 処理（Predict, Update, Postprocess）をヘッダー/ソースに分離

#### 3.1 ESKFState と ESKFRunner の定義（0.5日）

**作業内容**:
- `Inc/ESKF/eskf_state.hpp`: 統一された状態構造体定義
- `Inc/ESKF/eskf_runner.hpp`: メイン統合クラス定義
- `Src/ESKF/eskf_runner.cpp`: 実装

**ファイル例**:

```cpp
// Inc/ESKF/eskf_state.hpp
#pragma once

namespace eskf {

struct ESKFState {
    double p[3], v[3], q[4], ba[3], bg[3];
    double P[15*15];
    double Q_nominal[15*15];
    // ... その他メンバー
};

}  // namespace eskf
```

```cpp
// Inc/ESKF/eskf_runner.hpp
#pragma once

namespace eskf {

class ESKFRunner {
public:
    ESKFRunner();
    ~ESKFRunner();
    
    void init(const InitParams& params);
    void predict(ESKFState* s, const double* a_meas, const double* w_meas);
    void update(ESKFState* s, const char* type, const double* meas, int meas_len);
    void get_state(ESKFState* s, double* p, double* v, double* q, double* ba, double* bg, double* P);
    
private:
    SensorFilterLib filter_lib_;
};

}  // namespace eskf
```

**チェックリスト**:
- [ ] eskf_state.hpp 作成，メンバー確認
- [ ] eskf_runner.hpp 作成，メソッド確認
- [ ] インクルード依存グラフに従う

#### 3.2 Predict 処理の統合（0.5日）

**作業内容**:
- `MEX/mex_adaptive_predict.cpp` を `ESKFRunner::predict()` へ統合
- `MEX/mex_eskf_predict_postprocess.cpp` を `predict_postprocess()` へ統合

**統合対象**:
```cpp
// mex_adaptive_predict.cpp
- adaptive_q_scaling(...)      → Src/ESKF/eskf_runner.cpp

// mex_eskf_predict_postprocess.cpp
- predict_postprocess(...)     → Inc/ESKF/eskf_postprocess.hpp (宣言)
                                → Src/ESKF/eskf_postprocess.cpp (実装)
```

**実装**:
```cpp
// Src/ESKF/eskf_runner.cpp
void ESKFRunner::predict(ESKFState* s, const double* a_meas, const double* w_meas) {
    // 型変換: double → float
    Vector<3, float> p_f, v_f, a_meas_f, w_meas_f;
    // ...
    
    // ESKFCore::integrate_nominal() 呼び出し
    ESKFCore::integrate_nominal(...);
    
    // ESKFCore::predict_covariance() 呼び出し
    ESKFCore::predict_covariance(...);
    
    // predict_postprocess() 呼び出し
    predict_postprocess(...);
}
```

**チェックリスト**:
- [ ] ESKFRunner::predict() 実装
- [ ] predict_postprocess() 実装
- [ ] build_mex({'mex_run_eskf'}) でエラーなし
- [ ] verify_phase3() で Predict のみテスト

**コミット**:
```
commit: "refactor(phase3): Separate ESKF core layer"
```

---

### Phase 4: センサー更新層の統合（1.5日）

**目的**: Sensor Update（accel, gyro, mag, GPS, baro）を統合

#### 4.1 Sensor Preprocessor の統合（0.5日）

**作業内容**:
- `MEX/mex_sensor_preprocessor.cpp` を `sensor_preprocessor.hpp` へ統合
- 各センサーの前処理関数を実装

**統合対象**:
```cpp
preprocess_accel(...)   → Inc/Common/Sensor/sensor_preprocessor.hpp
preprocess_gyro(...)    → Inc/Common/Sensor/sensor_preprocessor.hpp
preprocess_mag(...)     → Inc/Common/Sensor/sensor_preprocessor.hpp
```

**チェックリスト**:
- [ ] sensor_preprocessor.hpp に関数定義
- [ ] 前処理ロジック確認

#### 4.2 Sensor Update の統合（1日）

**作業内容**:
- `MEX/mex_eskf_sensor_updates_full.cpp` を `ESKFRunner::update()` へ統合
- 各センサータイプ（accel, gyro, mag, GPS, baro）の更新処理を統合

**統合対象**:
```cpp
// mex_eskf_sensor_updates_full.cpp
- update_accel(...)     → ESKFRunner::update(...) の一部
- update_gyro(...)      → ESKFRunner::update(...) の一部
- update_mag(...)       → ESKFRunner::update(...) の一部
- update_gps(...)       → ESKFRunner::update(...) の一部
- update_baro(...)      → ESKFRunner::update(...) の一部
```

**実装**:
```cpp
// Src/ESKF/eskf_runner.cpp
void ESKFRunner::update(ESKFState* s, const char* type, 
                         const double* meas, int meas_len) {
    if (strcmp(type, "accel") == 0) {
        // accel 前処理 + 更新
        Vector<3, float> a_meas_f;
        for (int i = 0; i < 3; i++) {
            a_meas_f(i, 0) = static_cast<float>(meas[i]);
        }
        
        // Sensor Preprocessor
        PreprocessResult pre = preprocess_accel(a_meas_f, ...);
        
        // ESKF Update
        ESKFCore::update_accel(...);
    }
    else if (strcmp(type, "gps") == 0) {
        // GPS 更新
        // ...
    }
    // ... その他センサータイプ
}
```

**チェックリスト**:
- [ ] sensor_preprocessor の関数確認
- [ ] ESKFRunner::update() 実装
- [ ] build_mex({'mex_run_eskf'}) でエラーなし
- [ ] verify_phase4() で Predict + Update テスト

**コミット**:
```
commit: "refactor(phase4): Integrate sensor update layer"
```

---

### Phase 5: 統合・最適化・ドキュメント更新（1.5日）

**目的**: 全統合完了，最終テスト，ドキュメント更新

#### 5.1 mex_run_eskf.cpp の最終統合（0.5日）

**作業内容**:
- 不要な古い MEX ファイル参照を削除
- インクルード順序を最終確認
- グローバル変数を整理

**最終的な mex_run_eskf.cpp の構成**:
```cpp
#include <mex.h>
#include <cmath>
// ... 標準ライブラリ

// 依存順序
#include "../Inc/Common/Math/fixed_matrix.hpp"
#include "../Inc/Common/Math/vector_utils.hpp"
#include "../Inc/Common/Math/quaternion_lib.hpp"
#include "../Inc/ESKF/eskf_core.hpp"
#include "../Inc/ESKF/eskf_runner.hpp"
#include "../Inc/MEX/mex_type_conversion.hpp"

// MEX メイン
static std::map<uint64_t, ESKFState*> g_states;
static uint64_t g_next_handle = 1;

// MEX Interface Functions
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // 'init', 'step', 'get_state', 'free' の処理
}
```

**チェックリスト**:
- [ ] 古いコード削除
- [ ] インクルード順序確認
- [ ] グローバル変数整理

#### 5.2 古い MEX ファイルの削除（0.5日）

**作業内容**:
- 統合対象の MEX ファイルを削除（ git rm）
- `build_mex.m` をアップデート

**削除対象**:
```
MEX/
├── ❌ mex_adaptive_predict.cpp
├── ❌ mex_eskf_constructor.cpp
├── ❌ mex_eskf_predict_postprocess.cpp
├── ❌ mex_eskf_zupt.cpp
├── ❌ mex_filter_management.cpp
├── ❌ mex_quaternion_lib.cpp
├── ❌ mex_eskf_do_update.cpp
├── ❌ mex_eskf_sensor_updates_full.cpp
├── ❌ mex_sensor_preprocessor.cpp
└── ✓ mex_run_eskf.cpp (統合完了)
```

**build_mex.m の更新**:
```matlab
% 修正前
mex(..., 'MEX/mex_adaptive_predict.cpp', ...)
mex(..., 'MEX/mex_filter_management.cpp', ...)
% ...

% 修正後
mex(..., 'MEX/mex_run_eskf.cpp', ...)  % 統合済み
mex(..., 'MEX/mex_meukf_step_v2.cpp', ...)
mex(..., 'MEX/mex_sensor_filter.cpp', ...)
```

**コマンド**:
```bash
git rm MEX/mex_adaptive_predict.cpp
git rm MEX/mex_filter_management.cpp
# ... その他
```

**チェックリスト**:
- [ ] 不要な 9 ファイル削除
- [ ] build_mex.m 更新
- [ ] バイナリ (bin/) も同期

#### 5.3 ドキュメント・ローカル設定ファイルの整理（0.5日）

**作業内容**:
- 古い分析ドキュメント削除
- README を統合後の構造に更新
- CHANGELOG に統合内容を記載

**削除ドキュメント**:
```
MEX/
├── ❌ BINARY_SOURCE_COMPARISON.md
├── ❌ BUILD_FIXES.md
├── ❌ DEPENDENCY_ANALYSIS.md
├── ❌ MEX_FILES_ROLES.md
└── ...（計 10 ファイル削除）
```

**追加・更新ドキュメント**:
```
kalman/cpp/
├── README.md (更新) - 新構造の説明
├── INCLUDE_DEPENDENCY.md (新規) - インクルード依存
├── PROJECT_STRUCTURE.md (新規) - ファイル構造
├── CODE_REVIEW_CHECKLIST.md (新規) - レビューチェックリスト
├── build/
│   ├── INTEGRATION_PLAN.md (新規) - このドキュメント
│   ├── check_includes.m (新規)
│   └── verify_build.m (新規)
```

**チェックリスト**:
- [ ] 古いドキュメント削除
- [ ] README 更新
- [ ] CHANGELOG 記載
- [ ] ヘルプドキュメント確認

#### 5.4 最終ビルド・テスト（0.5日）

**作業内容**:
- 全 MEX ビルド確認
- 完全な回帰テスト実行

**ビルド**:
```matlab
build_mex({'mex_meukf_step_v2', 'mex_sensor_filter', 'mex_eskf_update_postprocess', 'mex_run_eskf'})
```

**テスト**:
```matlab
clear mex

% 1. 単一シード検証
run_simulation(42, true)

% 2. 複数シード検証
run_batch_10sets()

% 3. 既存実装との比較（該当する場合）
compare_mex_matlab_detailed()
```

**期待される結果**:
- ✓ 全 MEX ビルド成功
- ✓ MATLAB テストで NaN/Inf なし
- ✓ 数値精度 acceptable（<1e-4）
- ✓ エラーメッセージなし

**チェックリスト**:
- [ ] build_mex() 全成功
- [ ] run_simulation() 成功
- [ ] run_batch_10sets() 成功
- [ ] テスト結果を CSV で確認

#### 5.5 マージ・タグ付け（0時間）

**作業内容**: テスト成功後

```bash
# Feature ブランチ → main へマージ
git checkout main
git merge feature/integrate-mex-full

# タグ付け
git tag -a v1.0-integrated -m "MEX integration complete"

# リモートへプッシュ（オプション）
git push origin main
git push origin v1.0-integrated
```

**コミット**:
```
commit: "refactor(phase5): Complete MEX integration and documentation"
```

---

## 各フェーズの依存関係

```
Phase 1 (準備)
  ↓
Phase 2 (型・ユーティリティ)
  ↓
Phase 3 (ESKF コア)
  ↓
Phase 4 (Sensor Update)
  ↓
Phase 5 (統合・テスト)
```

各フェーズは**厳格に順序ありで実施**。スキップ不可。

---

## ビルド・テスト戦略

### テスト対象

| フェーズ | ビルド対象 | テスト項目 | 期待結果 |
|---------|----------|---------|--------|
| Phase 2 | meukf, sensor_filter | 既存 MEX 動作 | ✓ パス |
| Phase 3 | mex_run_eskf (Predict のみ) | Predict ステップ | ✓ 誤差 < 1e-6 |
| Phase 4 | mex_run_eskf (全機能) | Predict + Update | ✓ 誤差 < 1e-5 |
| Phase 5 | 全 MEX | 完全な run_simulation | ✓ 完全一致 |

### テストスクリプト例

```matlab
% verify_phase3.m
function ok = verify_phase3()
    % Phase 3: Predict 統合テスト
    
    fprintf('[TEST] Phase 3: Predict Integration\n');
    
    build_mex({'mex_run_eskf'});
    clear mex
    
    seed = 42;
    max_tol = 1e-6;
    
    try
        run_simulation(seed, false);
        
        % テスト結果を取得，比較
        % ... (実装詳細)
        
        fprintf('[PASS] Phase 3 test completed\n');
        ok = true;
        
    catch ME
        fprintf('[FAIL] %s\n', ME.message);
        ok = false;
    end
end
```

---

## リスク管理

### リスク #1: インクルード順序エラー

**影響度**: 🔴 致命的  
**発生確率**: 🟡 中程度（Phase 2-3）  
**対策**: 
- `check_includes.m` で自動チェック
- `TEMPLATE_TEST.hpp` でテンプレート展開確認

### リスク #2: テンプレート型の不一致

**影響度**: 🔴 致命的  
**発生確率**: 🟡 中程度（Phase 2）  
**対策**:
- `TEMPLATE_TEST.hpp` を先に実装・テスト
- 関数呼び出しでテンプレートパラメータを明示

### リスク #3: 数値精度の低下

**影響度**: 🟡 中程度  
**発生確率**: 🟢 低い  
**対策**:
- Phase 3-4 でテスト精度を厳格チェック
- 既存実装との比較

### リスク #4: MATLAB-C++ 間の型不一致

**影響度**: 🔴 致命的  
**発生確率**: 🟡 中程度（Phase 4-5）  
**対策**:
- `mex_type_conversion.hpp` を先に完成
- サンプルデータでラウンドトリップテスト

---

## チェックリスト：全フェーズ完了時の確認

```
最終チェック（Phase 5 終了時に実施）:

ビルド・テスト
- [ ] build_mex() で全メッセージが "OK"
- [ ] bin/ に 4 つの .mexw64 ファイルが存在
- [ ] run_simulation(42, true) でエラーなし
- [ ] run_batch_10sets() で 10 セット完走
- [ ] テスト結果が Results/ に CSV 出力

ドキュメント
- [ ] README が新構造を説明している
- [ ] INCLUDE_DEPENDENCY.md が完成
- [ ] PROJECT_STRUCTURE.md が完成
- [ ] CODE_REVIEW_CHECKLIST.md が完成
- [ ] build/ に verify_* スクリプトがある

コード品質
- [ ] インクルード順序が一貫している
- [ ] グローバル変数が最小化されている
- [ ] コメント・docstring が明確
- [ ] 定数が適切に #define または const で定義

Git
- [ ] コミット履歴が論理的（1 phase = 1 commit）
- [ ] コミットメッセージが明確
- [ ] タグが付けられている
- [ ] feature ブランチが main へマージされている

検証
- [ ] 既存テスト（MATLAB, Python など）が成功
- [ ] 数値精度が要求仕様を満たしている
- [ ] メモリ使用量が許容範囲内
- [ ] パフォーマンス劣化がない（むしろ向上）
```

---

## 参考ドキュメント

- [01_COMMIT_CHANGES_SUMMARY.md](01_COMMIT_CHANGES_SUMMARY.md) - error ブランチの変更内容
- [02_FAILURE_ROOT_CAUSE_ANALYSIS.md](02_FAILURE_ROOT_CAUSE_ANALYSIS.md) - ビルド失敗の根本原因
- [03_PREVENTION_STRATEGIES.md](03_PREVENTION_STRATEGIES.md) - 再発防止策

---

## Q&A

### Q. なぜ Phase 1 に 1 日かけるのか？

**A.** 十分な準備がないと、Phase 2-4 でパニック状態になります。
ドキュメント・スクリプト・テスト環境があれば、以降の段階が **20% 高速** になります。

### Q. Phase をスキップできるか？

**A.** **いいえ**。各フェーズは厳格な順序で、各々依存関係があります。
スキップすると、後の段階でトラブルが発生します。

### Q. ビルド失敗時はどうするか？

**A.** 
1. `build_mex_log_YYYYMMDD_HHMMSS.txt` を確認
2. 該当フェーズの修正を実施
3. その フェーズのテストを再実行
4. 前段階へのロールバックは最後の手段

### Q. 予定より遅れた場合は？

**A.** リスク缓衝（20%）があります。
- 1-2 日の遅延なら許容範囲
- 3 日以上遅延 → 計画見直し（フェーズの統合など）

---

## 参考リンク

- [README.md](../README.md)
- [build_mex.m](../build/build_mex.m)
- [run_simulation.m](../../run_simulation.m)
