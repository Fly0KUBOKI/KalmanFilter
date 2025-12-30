# 再発防止策（チェックリスト＆ガイドライン）

**生成日**: 2025-12-30  
**目的**: 大規模統合・分離リファクタリング時のビルド失敗を防止

---

## 再発防止の原則

```
┌─────────────────────────────────────────────────────────────┐
│ 原則 1: 「小さく、段階的に」統合する                          │
│ 原則 2: 各段階で完全にビルド・テストする                      │
│ 原則 3: ファイル構造を可視化し、依存関係を把握する             │
│ 原則 4: インクルード順序は必ずテストする                      │
│ 原則 5: 自動ビルドスクリプトで検証を自動化する               │
└─────────────────────────────────────────────────────────────┘
```

---

## 再発防止策 #1: インクルード順序の統一とテスト

### 問題: インクルード順序は「運ゲー」になりやすい

**今回のエラーの主原因**:
```cpp
// ❌ 悪い例: fixed_matrix が遅い
#include "quaternion_lib.hpp"      // quaternion_lib が fixed_matrix を必要とする
#include "vector_utils.hpp"        // vector_utils も fixed_matrix を必要とする
#include "fixed_matrix.hpp"        // ← 依存元が遅い
```

**✓ 良い例: 依存元を先に配置**
```cpp
#include "fixed_matrix.hpp"        // ← 基礎型定義（最初）
#include "vector_utils.hpp"        // fixed_matrix を利用
#include "quaternion_lib.hpp"      // fixed_matrix を利用
```

### 対応策: インクルード順序ガイドライン

#### ステップ 1: インクルード依存グラフの作成

**ファイル**: `kalman/cpp/INCLUDE_DEPENDENCY.md` (新規作成)

```markdown
# インクルード依存関係マップ

## レイヤー構造

### レイヤー 1: 最基礎（依存なし）
- fixed_matrix.hpp         ← 基本型定義
- quaternion.hpp           ← 基本クォータニオン
- statistics.hpp           ← 統計関数

### レイヤー 2: レイヤー1に依存
- vector_utils.hpp         ← fixed_matrix を使用
- matrix_utils.hpp         ← fixed_matrix を使用
- quaternion_lib.hpp       ← fixed_matrix を使用

### レイヤー 3: レイヤー1-2に依存
- sensor_preprocessor.hpp  ← vector_utils, matrix_utils を使用
- eskf_core.hpp            ← すべてを使用

### レイヤー 4: 高レベル（MEX バインディング）
- mex_type_conversion.hpp  ← すべてに依存
- mex_run_eskf.cpp         ← すべてに依存
```

#### ステップ 2: インクルード順序チェックスクリプトの追加

**ファイル**: `kalman/cpp/build/check_includes.m` (新規作成)

```matlab
function ok = check_includes(cpp_file)
% インクルード順序をチェック
% 返値: true = OK, false = エラー

  fid = fopen(cpp_file, 'r');
  line_num = 0;
  layer = 0;
  
  while true
      line = fgetl(fid);
      if ~ischar(line), break; end
      line_num = line_num + 1;
      
      if ~contains(line, '#include'), continue; end
      
      % レイヤーを判定
      if contains(line, 'fixed_matrix.hpp')
          new_layer = 1;
      elseif contains(line, {'vector_utils', 'matrix_utils', 'quaternion_lib'})
          new_layer = 2;
      elseif contains(line, {'sensor_preprocessor', 'eskf_core'})
          new_layer = 3;
      else
          new_layer = 4;
      end
      
      % チェック: レイヤーは単調増加
      if new_layer < layer
          fprintf('[ERROR] インクルード順序: Line %d が逆順\n', line_num);
          fprintf('  %s\n', line);
          ok = false;
          fclose(fid);
          return;
      end
      layer = new_layer;
  end
  
  fclose(fid);
  ok = true;
end
```

---

## 再発防止策 #2: 段階的な統合計画と マイルストーン

### 問題: 「すべて一度に」統合しようとした

**失敗パターン**:
```
1回のコミット で 9個の MEX を統合
  → ビルド失敗時に何が原因かわからない
  → ロールバックも git bisect も困難
```

### 対応策: 統合計画（Phase ベース）

#### フェーズ計画テンプレート

**ファイル**: `kalman/cpp/INTEGRATION_PHASES.md` (新規作成)

```markdown
# MEX 統合フェーズ計画

## Phase 1: 基礎型ライブラリの分離（1日）
- 目標: fixed_matrix, quaternion_lib などを Inc/Lib に分離
- ビルド対象: mex_meukf_step_v2, mex_sensor_filter
- 期待: 既存 2 MEX がビルド＆テスト成功
- コミット: 1 個

## Phase 2: Utility 関数の統合（1日）
- 統合対象: mex_quaternion_lib, mex_filter_management
- 方法: 既存 MEX を mexCallMATLAB で呼び出し（段階的に C++ 実装へ）
- ビルド対象: mex_run_eskf（部分実装）
- テスト: run_simulation で動作確認
- コミット: 1-2 個

## Phase 3: Predict 統合（1日）
- 統合対象: mex_adaptive_predict, mex_eskf_predict_postprocess
- 方法: ESKFRunner クラスに predict() メソッド実装
- ビルド対象: mex_run_eskf（predict のみ）
- テスト: Predict ステップのみ検証
- コミット: 1 個

## Phase 4: Update 統合（1-2日）
- 統合対象: mex_eskf_do_update, mex_eskf_sensor_updates_full
- 方法: ESKFRunner クラスに update() メソッド実装
- ビルド対象: mex_run_eskf（update のみ）
- テスト: Predict + Update の検証
- コミット: 2-3 個

## Phase 5: 完全統合テスト（1日）
- 全ステップ統合: init + predict + update + postprocess
- テスト: run_simulation（全 seed）, run_batch_10sets
- ドキュメント更新
- コミット: 1 個
```

#### 各フェーズでの ビルド・テストチェックリスト

```markdown
## ビルド・テストチェックリスト

### ビルド確認
- [ ] `build_mex()` でエラーなし
- [ ] MEX ファイルが `bin/` に生成されている
- [ ] `clear mex` で前バージョンをアンロード

### MATLAB テスト
- [ ] `run_simulation(42, true)` で基本動作確認
- [ ] エラー/警告がないこと
- [ ] 数値結果が妥当（NaN, Inf がない）

### 数値検証（該当フェーズのみ）
- [ ] Phase 3: Predict の数値誤差 < 1e-6
- [ ] Phase 4: Update の数値誤差 < 1e-5
- [ ] Phase 5: 全体での結果が既存実装と一致

### ドキュメント
- [ ] コミットメッセージが明確
- [ ] CHANGELOG が更新されている
```

---

## 再発防止策 #3: 型安全性の強化（静的チェック）

### 問題: テンプレート型が実行時まで検出されない

**解決方法**: ビルドスクリプトに型チェック機能を追加

#### 方法 A: コンパイル時にテンプレート展開を強制

```cpp
// kalman/cpp/Inc/TEMPLATE_TEST.hpp (新規作成)

#pragma once

// テンプレートの型を事前に展開して、コンパイル時エラーを検出
#include "Common/Math/fixed_matrix.hpp"
#include "Common/Math/vector_utils.hpp"
#include "Common/Math/quaternion_lib.hpp"
#include "MEX/mex_type_conversion.hpp"

namespace template_test {

// テンプレート明示的な展開（コンパイル時に型チェック）
void force_template_instantiation() {
    // Vector 型の展開
    cmath_fx::Vector<3, float> v3f;
    cmath_fx::Vector<4, float> v4f;
    cmath_fx::Vector<15, float> v15f;
    
    // Matrix 型の展開
    cmath_fx::Matrix<3, 3, float> m33f;
    cmath_fx::Matrix<15, 15, float> m1515f;
    
    // vector_utils の展開
    float n3 = norm3(v3f.data());
    
    // mex_type_conversion の展開（コンパイル時に関数定義を確認）
    const mxArray* dummy = nullptr;
    cmath_fx::Vector<3, float> test_v3f;
    mex_conv::matToVector<3>(dummy, test_v3f);
    
    cmath_fx::Matrix<15, 15, float> test_m1515f;
    mex_conv::matToMatrix<15, 15>(dummy, test_m1515f);
}

}  // namespace template_test
```

#### 方法 B: ビルドスクリプトに型チェック段階を追加

```matlab
% kalman/cpp/build/build_mex.m (修正)

function build_mex(targets)
    % ... 既存コード ...
    
    % 新規: テンプレート型の事前チェック
    fprintf('[INFO] Template type checking...\n');
    if ~check_template_types()
        error('Template type check failed');
    end
    
    % ... コンパイル処理 ...
end

function ok = check_template_types()
    % コンパイル用テストファイルを作成して、
    % インクルードとテンプレート展開をテストする
    
    test_file = 'test_templates.cpp';
    fid = fopen(test_file, 'w');
    fprintf(fid, '#include "../Inc/TEMPLATE_TEST.hpp"\n');
    fprintf(fid, 'int main() { template_test::force_template_instantiation(); }\n');
    fclose(fid);
    
    % コンパイル試行
    cmd = sprintf('cl /c /I"." %s 2>&1', test_file);
    [status, output] = system(cmd);
    
    ok = (status == 0);
    
    % テストファイル削除
    delete(test_file);
    if isfile('test_templates.obj')
        delete('test_templates.obj');
    end
end
```

---

## 再発防止策 #4: 自動回帰テスト（CI/CD）

### 問題: 「ビルドに成功したから OK」では不十分

**解決方法**: 各フェーズでの自動テスト

#### スクリプト: `kalman/cpp/build/verify_build.m` (新規作成)

```matlab
function success = verify_build(mex_name, varargin)
% MEX のビルド・テストを自動化
% 用法: verify_build('mex_run_eskf', 'seed', 42)

    % ビルド
    fprintf('[BUILD] Compiling %s...\n', mex_name);
    build_mex({mex_name});
    
    if ~isfile(fullfile('bin', [mex_name, '.mexw64']))
        error('Build failed: MEX file not generated');
    end
    fprintf('[OK] %s compiled successfully\n\n', mex_name);
    
    % MATLAB テスト
    fprintf('[TEST] Running MATLAB test...\n');
    clear mex
    
    try
        % 基本テスト: run_simulation
        seed = 42;
        if ~isempty(varargin) && strcmp(varargin{1}, 'seed')
            seed = varargin{2};
        end
        
        fprintf('  Running simulation with seed=%d...\n', seed);
        run_simulation(seed, false);
        
        fprintf('[OK] MATLAB test passed\n\n');
        success = true;
        
    catch ME
        fprintf('[ERROR] MATLAB test failed:\n');
        fprintf('  %s\n', ME.message);
        success = false;
    end
end
```

#### ビルドスクリプトへの統合

```matlab
% build_mex.m に追加

function build_mex(targets, varargin)
    % ... 既存コード ...
    
    % 新規: 各ターゲットのテスト
    verbose = any(strcmp(varargin, 'verbose'));
    
    for i = 1:length(targets)
        target = targets{i};
        fprintf('\n=== Building and testing %s ===\n', target);
        
        if ~verify_build(target)
            if ~verbose
                error('Build/test failed: %s', target);
            end
        end
    end
end
```

---

## 再発防止策 #5: ドキュメント・ファイル構造の可視化

### 問題: ファイル構造が複雑すぎて、統合時に見落とされた

**解決方法**: ファイル構造と依存関係を可視化・ドキュメント化

#### ファイル: `kalman/cpp/PROJECT_STRUCTURE.md` (新規作成)

```markdown
# プロジェクト構造図

## ディレクトリ構造

```
kalman/cpp/
├── Inc/                          ← ヘッダー（インターフェース）
│   ├── ESKF/
│   │   ├── eskf_core.hpp         ← ESKF コア計算
│   │   ├── eskf_runner.hpp       ← 統合フレームワーク
│   │   ├── eskf_initializer.hpp  ← 初期化
│   │   └── eskf_postprocess.hpp  ← 後処理
│   ├── Common/
│   │   ├── Math/
│   │   │   ├── fixed_matrix.hpp  ← 基本型（レイヤー 1）
│   │   │   ├── vector_utils.hpp  ← ユーティリティ（レイヤー 2）
│   │   │   └── quaternion_lib.hpp← クォータニオン（レイヤー 2）
│   │   ├── Sensor/
│   │   │   ├── sensor_filter.hpp ← センサーフィルタ
│   │   │   └── sensor_preprocessor.hpp ← 前処理
│   │   └── filter_management.hpp ← フィルター管理
│   └── MEX/
│       ├── mex_type_conversion.hpp ← 型変換
│       └── mex_type_conversion.cpp ← 実装
│
├── Src/                          ← 実装（.cpp）
│   ├── ESKF/
│   │   ├── eskf_core.cpp
│   │   ├── eskf_runner.cpp
│   │   └── eskf_postprocess.cpp
│   └── Common/
│       └── ...
│
├── MEX/                          ← MEX バインディング
│   ├── mex_run_eskf.cpp          ← 統合メイン
│   ├── mex_meukf_step_v2.cpp     ← 別系統（独立）
│   └── mex_sensor_filter.cpp     ← センサーフィルタ
│
├── build/                        ← ビルドスクリプト
│   ├── build_mex.m
│   ├── verify_build.m
│   ├── check_includes.m
│   └── ...
```

## 依存関係グラフ

```
[レイヤー 1: 基本型]
  ├─ fixed_matrix.hpp
  ├─ quaternion.hpp
  └─ statistics.hpp
        ↓
[レイヤー 2: ユーティリティ]
  ├─ vector_utils.hpp
  ├─ matrix_utils.hpp
  ├─ quaternion_lib.hpp
  └─ (sensor_preprocessor.hpp)
        ↓
[レイヤー 3: ESKF コア]
  ├─ eskf_core.hpp
  ├─ sensor_filter.hpp
  └─ filter_management.hpp
        ↓
[レイヤー 4: 統合層]
  ├─ eskf_runner.hpp
  └─ mex_type_conversion.hpp
        ↓
[レイヤー 5: MEX]
  ├─ mex_run_eskf.cpp
  ├─ mex_meukf_step_v2.cpp
  └─ mex_sensor_filter.cpp
```

## インクルード標準順序

```cpp
// ファイルの先頭に記載すべきインクルード順序

// 1. MEX ヘッダー
#include <mex.h>

// 2. 標準ライブラリ
#include <cmath>
#include <cstring>
#include <vector>
#include <map>

// 3. レイヤー 1: 基本型
#include "../Inc/Common/Math/fixed_matrix.hpp"

// 4. レイヤー 2: ユーティリティ
#include "../Inc/Common/Math/vector_utils.hpp"
#include "../Inc/Common/Math/quaternion_lib.hpp"

// 5. レイヤー 3: ESKF コア
#include "../Inc/ESKF/eskf_core.hpp"
#include "../Inc/Common/Sensor/sensor_filter.hpp"

// 6. レイヤー 4: 統合層
#include "../Inc/ESKF/eskf_runner.hpp"
#include "../Inc/MEX/mex_type_conversion.hpp"

// 7. その他
#include <algorithm>
```
```

---

## 再発防止策 #6: コードレビュー・チェックリスト

### ファイル: `kalman/cpp/CODE_REVIEW_CHECKLIST.md` (新規作成)

```markdown
# 大規模統合リファクタリング コードレビューチェックリスト

## インクルード順序（必須）
- [ ] レイヤー 1（fixed_matrix など）が最初
- [ ] レイヤー 2（vector_utils など）が次
- [ ] 逆順のインクルード（#include ... の順序が下降）がない
- [ ] check_includes.m で自動検証

## テンプレート型（必須）
- [ ] 型定義が すべての場所で認識されている
- [ ] Vector<N, T>, Matrix<N, M, T> の使用に誤りがない
- [ ] テンプレートパラメータが明示的に指定されている
- [ ] TEMPLATE_TEST.hpp でテスト

## スコープと生存期間（必須）
- [ ] グローバル変数の使用を最小化
- [ ] ローカル変数が正しいスコープで定義されている
- [ ] 参照(&)の生存期間が正しい
- [ ] static 変数の初期化が適切

## 関数シグネチャ（必須）
- [ ] 関数宣言と定義が一致
- [ ] 呼び出し側と定義側の引数型が一致
- [ ] テンプレート関数の場合、パラメータが推論可能か確認
- [ ] オーバーロード関数がある場合、曖昧性がない

## ビルド・テスト（必須）
- [ ] `build_mex()` でエラーなし
- [ ] `verify_build()` で自動テスト成功
- [ ] `run_simulation()` で基本動作確認
- [ ] 既存テスト（`run_batch_10sets` など）が成功

## ドキュメント（推奨）
- [ ] 新しい関数・クラスにコメント追加
- [ ] 大規模な変更は CHANGELOG に記載
- [ ] 統合後の API 変更を MATLAB 側で反映

## コミット（推奨）
- [ ] 1 つのコミットは 1 つの論理的な変更（1 つのフェーズ）
- [ ] コミットメッセージが明確（「何を」「なぜ」が分かる）
- [ ] `git bisect` で問題の特定が可能な粒度
```

---

## 再発防止策 #7: Git ワークフロー・ブランチ戦略

### 推奨: Feature ブランチ戦略 + rebase

```bash
# 1. 作業ブランチ を作成
git checkout -b feature/integrate-mex-phase1

# 2. 小さなコミットを積み重ねる
git add kalman/cpp/Inc/ESKF/eskf_state.hpp
git commit -m "feat: Add ESKFState structure (Phase 1)"

git add kalman/cpp/MEX/mex_run_eskf.cpp
git commit -m "refactor: Integrate quaternion functions (Phase 1)"

# 3. 定期的にテスト
# → build_mex(), run_simulation() で確認

# 4. main へマージ
git rebase main
git checkout main
git merge feature/integrate-mex-phase1

# 5. 問題が発生したら、git bisect で特定
git bisect start
git bisect bad HEAD
git bisect good v1.0
# → 各段階でテストして、問題のコミットを特定
```

---

## チェックリスト：統合リファクタリング前に確認

```
これから大規模な MEX 統合リファクタリング を行う前に、以下をチェック：

[ ] インクルード依存グラフが作成されている（INCLUDE_DEPENDENCY.md）
[ ] 統合フェーズ計画が完成している（INTEGRATION_PHASES.md）
[ ] テンプレートテスト機能がある（TEMPLATE_TEST.hpp, check_template_types）
[ ] インクルード順序チェック スクリプトがある（check_includes.m）
[ ] 自動テスト機能がある（verify_build.m）
[ ] ドキュメントが完成している（PROJECT_STRUCTURE.md）
[ ] コードレビューチェックリストがある（CODE_REVIEW_CHECKLIST.md）
[ ] main ブランチが clean 状態（未コミット変更がない）
[ ] feature ブランチを作成予定（git flow workflow）

すべてチェック済み → 統合開始 OK
```

---

## 参考

- [04_INTEGRATION_REFACTORING_PLAN.md](04_INTEGRATION_REFACTORING_PLAN.md) - 統合・分離計画書（詳細）
- [01_COMMIT_CHANGES_SUMMARY.md](01_COMMIT_CHANGES_SUMMARY.md) - コミット変更内容
- [02_FAILURE_ROOT_CAUSE_ANALYSIS.md](02_FAILURE_ROOT_CAUSE_ANALYSIS.md) - 根本原因分析
