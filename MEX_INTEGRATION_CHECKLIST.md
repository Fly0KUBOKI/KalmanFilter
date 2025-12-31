# MEX 統合実行チェックリスト

**使用方法**: 各Phaseを実行する前にこのドキュメントをコピーしてチェックを入れながら進める

---

## Phase 0: 準備フェーズ

### ✓ Step 0.1: 現在のバージョン確認
```
□ git log --oneline -5 で履歴確認
□ 現在のコミットハッシュをメモ: ________________

結果:
```

### ✓ Step 0.2: バックアップコミット作成
```
□ git status で clean な状態か確認
□ git add -A
□ git commit -m "[backup] 統合前のクリーンな状態"
□ git stash create "phase0-baseline" を実行
□ git stash list でバックアップが存在することを確認

コミットメッセージ: ________________________________
バックアップハッシュ: ________________________________
```

### ✓ Step 0.3: 単体テスト＆ベースライン記録
```
□ cd kalman && pwd で正確なパスを確認
□ clear mex を実行（古いMEXキャッシュをクリア）
□ run_simulation(42, true) を実行
□ 結果: ___________________________________________
   - ✓ 成功 / × 失敗
   - NaN/Inf の有無: _____________________________
   - estimation_01.csv の行数: ____________________

□ estimation_01.csv を estimation_phase0.csv として保存
   cp kalman/Results/estimation_01.csv estimation_phase0.csv
```

### ✓ Step 0.4: 完了確認
```
□ すべてのステップが完了か確認
□ 復旧が可能な状態か確認（git stash で復旧可能）

Phase 0 完了日時: ____________________________
```

---

## Phase 1: mex_meukf_step_v2 統合フェーズ

### ✓ Step 1.1: 源ファイル分析
```
□ kalman/cpp/MEX/mex_meukf_step.cpp を確認
  主な関数: _________________________________________
  
□ kalman/cpp/src/MEUKF/meukf_core.cpp を確認
  主な関数: _________________________________________

□ 入力パラメータを確認: _________________________________
□ 出力パラメータを確認: _________________________________

分析完了: □ Yes □ No
疑問点: ___________________________________________
```

### ✓ Step 1.2: C++ 実装（メモ）
```
□ mex_run_eskf.cpp を編集開始
  - #include "../src/MEUKF/meukf_core.h" を追加: □
  - MEUKFStep クラスを実装: □
  - mexFunction に meukf_step 処理を追加: □

実装完了: □ Yes □ No
疑問点: ___________________________________________
```

### ✓ Step 1.3: ビルド設定更新
```
□ build_mex.m を編集
  - meukf_core_cpp パスを設定: □
  - mex_run_eskf のリンク設定に meukf_core_cpp を追加: □

変更内容:
────────────────────────────────────────────
(build_mex.m から該当部分をコピー)
────────────────────────────────────────────
```

### ✓ Step 1.4: ビルド実行
```
□ cd kalman/cpp/build
□ matlab -batch "build_mex({'mex_run_eskf'})" を実行
□ ビルド結果:
  - 成功: □ / 失敗: □
  - エラーメッセージ: _____________________________
  
□ ls -lh ../bin/mex_run_eskf.mexw64 でサイズ確認
  ファイルサイズ: ______ KB (前回: ______ KB)
  増加量: ______ KB （機能追加なので増えるべき）
```

### ✓ Step 1.5: テスト - MEUKF 単体確認
```
□ test_meukf_integration.m を実行
  - 旧 mex_meukf_step.mexw64 での実行: □
    結果: state_diff = ______, cov_diff = ______
  
  - 新 mex_run_eskf での実行: □
    結果: state_diff = ______, cov_diff = ______
  
  - 差分確認: □
    判定: □ Pass（< 1e-10） □ Fail（≥ 1e-10）
    
テスト結果:
  ✓ Pass: MEUKF統合成功
  ✗ Fail: 原因調査が必要 → Step 1.2 へ戻る
```

### ✓ Step 1.6: バックアップ＆コミット
```
□ git status で変更ファイル確認
  変更ファイル:
  - kalman/cpp/MEX/mex_run_eskf.cpp
  - kalman/cpp/build/build_mex.m
  - kalman/cpp/bin/mex_run_eskf.mexw64

□ git add kalman/cpp/MEX/mex_run_eskf.cpp
□ git add kalman/cpp/build/build_mex.m
□ git add kalman/cpp/bin/mex_run_eskf.mexw64

□ git commit -m "[Phase 1] MEUKF機能をmex_run_eskfに統合..."

□ git stash create "phase1-meukf-integrated"
□ git stash list でバックアップ確認

Phase 1 完了日時: ____________________________
コミットハッシュ: ____________________________
```

---

## Phase 2: mex_sensor_filter 統合フェーズ

### ✓ Step 2.1: 源ファイル分析
```
□ kalman/cpp/MEX/mex_sensor_filter.cpp を確認
  主な関数: _________________________________________

□ kalman/cpp/include/Common/Sensor/sensor_filter.hpp を確認
  クラス/構造体: ______________________________________

分析完了: □ Yes □ No
```

### ✓ Step 2.2: C++ 実装
```
□ mex_run_eskf.cpp を編集
  - #include "../include/Common/Sensor/sensor_filter.hpp" 追加: □
  - SensorFilterComponent クラス実装: □
  - mexFunction に sensor_filter 処理追加: □

実装完了: □ Yes □ No
```

### ✓ Step 2.3: ビルド設定更新
```
□ build_mex.m を編集
  - sensor_filter_cpp パスを設定: □
  - リンク設定に追加: □

変更確認: □ Yes □ No
```

### ✓ Step 2.4: ビルド実行
```
□ build_mex({'mex_run_eskf'})

ビルド結果:
  - 成功: □ / 失敗: □
  - ファイルサイズ変化: ______ KB → ______ KB
```

### ✓ Step 2.5: テスト - センサーフィルタ確認
```
□ test_sensor_filter_integration.m を実行

テスト結果:
  - 旧 mex_sensor_filter での実行: □
  - 新 mex_run_eskf での実行: □
  - 差分確認: filtered_diff = _______
  
判定:
  □ Pass（< 1e-10）
  □ Fail（≥ 1e-10）→ 原因調査
```

### ✓ Step 2.6: バックアップ＆コミット
```
□ git add -A
□ git commit -m "[Phase 2] センサーフィルタをmex_run_eskfに統合..."
□ git stash create "phase2-sensor-filter-integrated"

Phase 2 完了日時: ____________________________
コミットハッシュ: ____________________________
```

---

## Phase 3: 統合検証フェーズ

### ✓ Step 3.1: 初期化コード確認
```
□ kalman/run_batch_10sets.m を確認
  初期化コード存在: □ Yes □ No
  
  確認内容:
  ──────────────────────────────────────────
  (確認した初期化コード)
  ──────────────────────────────────────────
```

### ✓ Step 3.2: MATLAB ラッパー確認
```
□ kalman/ESKF/@ESKF/ESKF.m を確認
  - mex_run_eskf 呼び出しパターン: __________________
  - パラメータ型チェック: □ Yes □ No
  - 初期化順序: _____________________________________
```

### ✓ Step 3.3: 単体テスト
```
□ cd kalman
□ clear mex
□ run_simulation(42, true)

テスト結果:
  - 完了: □ Yes □ No
  - NaN/Inf エラー: □ なし □ あり
  - 位置更新: □ あり （全てゼロではない）□ なし
  
□ estimation_phase3.csv として結果保存
```

### ✓ Step 3.4: 非回帰テスト
```
□ test_regression_phase3.m を実行

テスト結果:
  - Phase 0（独立MEX）での実行: □
  - Phase 3（統合版）での実行: □
  
差分分析:
  - Max difference: _______
  - Mean difference: _______
  
判定:
  □ Pass（max_diff < 1e-4）
  □ Fail（max_diff ≥ 1e-4）→ 原因調査
```

### ✓ Step 3.5: バッチテスト
```
□ cd kalman
□ clear mex
□ run_batch_10sets()

バッチテスト結果:
  - 成功: ___ / 10
  - NaN/Inf 検出回数: _____
  - 平均推定誤差: _________
  
判定:
  □ Pass（10/10 成功、NaN/Inf なし）
  □ Fail（失敗あり）→ 原因調査
```

### ✓ Step 3.6: バックアップ＆コミット
```
□ git add -A
□ git commit -m "[Phase 3] 統合ESKF検証完了 - run_batch_10sets() で10/10成功"
□ git stash create "phase3-integration-verified"

Phase 3 完了日時: ____________________________
```

---

## Phase 4: 統合確定フェーズ

### ✓ Step 4.1: ラッパー置き換え
```
□ kalman/ESKF/@ESKF/ESKF.m を編集
  - 3つのMEX呼び出しを mex_run_eskf に統一: □

確認:
  Before: _____________________________________________
  After: ______________________________________________
```

### ✓ Step 4.2: ビルドスクリプト整理
```
□ kalman/cpp/build/build_mex.m を編集
  - 旧MEX設定をコメント化: □

確認:
  - mex_meukf_step ビルド設定: コメント化済み □
  - mex_sensor_filter ビルド設定: コメント化済み □
```

### ✓ Step 4.3: 最終テスト
```
□ cd kalman
□ clear mex
□ run_batch_10sets()

結果:
  - 成功: ___ / 10
  - 判定: □ Pass □ Fail
```

### ✓ Step 4.4: コミット＆タグ
```
□ git add -A
□ git commit -m "[Phase 4] 統合ESKF確定 - mex_run_eskf へ完全統合"
□ git tag -a "v1.0-integrated-mex" -m "..."
□ git stash create "phase4-integration-final"

Phase 4 完了日時: ____________________________
```

---

## Phase 5: 後始末フェーズ（1週間後）

### ✓ Step 5.1: テスト期間確認
```
□ Phase 4 完了から 1 週間以上経過: □ Yes □ No
□ その間、run_batch_10sets() で 5 回以上成功: □ Yes □ No
```

### ✓ Step 5.2: 旧MEXバイナリ削除
```
□ git rm kalman/cpp/bin/mex_meukf_step_v2.mexw64
□ git rm kalman/cpp/bin/mex_sensor_filter.mexw64
□ git status で削除が反映: □ Yes □ No
```

### ✓ Step 5.3: ビルド設定から削除
```
□ build_mex.m から旧MEX設定コメント部分を削除
   （コメント → 削除）

確認: □ Yes □ No
```

### ✓ Step 5.4: 最終コミット
```
□ git add -A
□ git commit -m "[cleanup] 統合完了後、旧MEXバイナリを削除"
□ git stash create "phase5-cleanup-final"

Phase 5 完了日時: ____________________________
```

---

## トラブルシューティング記録

### エラーが発生した場合の記録テンプレート

#### ケース: ________________________

**発生フェーズ**: Phase __

**エラー内容**:
```
(エラーメッセージをコピー)
```

**原因推定**:
```
(考えられる原因)
```

**試行内容**:
```
(試した対応)
```

**結果**: □ 解決 □ 未解決 → ロールバック

**ロールバック手順**:
```
git stash list
git stash apply phase○-***
git status で復旧確認
run_batch_10sets() で動作確認
```

---

## 総合チェックリスト（全Phase）

### 準備状態
- [ ] git 環境セットアップ済み
- [ ] バックアップが取得可能
- [ ] テスト方法を理解

### 実装段階
- [ ] Phase 0 完了
- [ ] Phase 1 完了
- [ ] Phase 2 完了
- [ ] Phase 3 完了
- [ ] Phase 4 完了

### 検証
- [ ] 非回帰テスト合格（差分 < 1e-4）
- [ ] バッチテスト合格（10/10 成功）
- [ ] NaN/Inf エラーなし

### 記録
- [ ] コミットメッセージが詳細
- [ ] バックアップポイント（git stash）が複数あり
- [ ] テスト結果が記録されている

---

## 完了サイン

| 項目 | サイン | 日時 |
|------|--------|------|
| Phase 0 完了 | ________ | __________ |
| Phase 1 完了 | ________ | __________ |
| Phase 2 完了 | ________ | __________ |
| Phase 3 完了 | ________ | __________ |
| Phase 4 完了 | ________ | __________ |
| Phase 5 完了 | ________ | __________ |

---

**このチェックリストは印刷またはコピーして、各Phaseごとに進捗を記録してください。**

**重要**: エラーが発生した場合は、無理に進めずに git stash で復旧してから原因を分析してください。
