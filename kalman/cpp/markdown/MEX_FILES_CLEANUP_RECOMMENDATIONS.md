# MEXフォルダ整理 推奨事項

## 調査結果サマリー

- **総ファイル数**: 約40ファイル（.cpp）
- **ビルド対象**: 約30ファイル
- **ビルドスキップ**: 2ファイル
- **ビルド対象外**: 約8ファイル
- **ソースなしバイナリ**: 3ファイル（binフォルダ）

## 即座に実行すべきアクション

### 1. 古いバイナリの削除

#### MEXフォルダ内のバイナリ（すべて削除）
```bash
# 以下のファイルを削除
kalman/cpp/MEX/mex_eskf_predict_postprocess.mexw64
kalman/cpp/MEX/mex_eskf_update_postprocess.mexw64
kalman/cpp/MEX/mex_eskf_zupt.mexw64
kalman/cpp/MEX/mex_eskf_reset.mexw64
```

**理由:** ソースコードが削除されたか、古いビルド残骸

#### binフォルダ内の古いバイナリ
```bash
# 以下のファイルを削除
kalman/cpp/bin/mex_eskf_run.mexw64          # 正しくはmex_run_eskf.mexw64
kalman/cpp/bin/mex_meukf_step.mexw64        # 正しくはmex_meukf_step_v2.mexw64
```

**理由:** 名前不一致、古いビルド残骸

### 2. ビルドスクリプトの更新

#### 追加すべきファイル

**mex_quaternion_lib.cpp** - `mex_matlab_helpers.m`で使用されている
```matlab
% mex_quaternion_lib (mex_matlab_helpers.mで使用)
if exist('mex_quaternion_lib.cpp', 'file')
    if wants('mex_quaternion_lib') && build_single_mex('mex_quaternion_lib.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
        built_count = built_count + 1;
    end
end
```

## 調査が必要なファイル

### 使用状況を確認すべきファイル

以下のファイルはソースが存在するが、使用箇所が不明です。
コードベース全体を検索して使用状況を確認してください。

1. **mex_kalman_compute.cpp**
   - バイナリ: ✅ 存在
   - ビルド: ❌ 対象外
   - 検索コマンド: `grep -r "mex_kalman_compute" kalman/`

2. **mex_common_lib.cpp**
   - バイナリ: ❌ なし
   - ビルド: ❌ 対象外
   - 検索コマンド: `grep -r "mex_common_lib" kalman/`

3. **mex_filter_utils.cpp**
   - バイナリ: ❌ なし
   - ビルド: ❌ 対象外
   - 検索コマンド: `grep -r "mex_filter_utils" kalman/`

4. **mex_eskf_helper.cpp**
   - バイナリ: ❌ なし
   - ビルド: ❌ 対象外
   - 検索コマンド: `grep -r "mex_eskf_helper" kalman/`

5. **mex_eskf_core_v2.cpp**
   - バイナリ: ❌ なし
   - ビルド: ❌ 対象外
   - 検索コマンド: `grep -r "mex_eskf_core_v2" kalman/`

6. **mex_kf_core.cpp**
   - バイナリ: ❌ なし
   - ビルド: ❌ 対象外
   - 検索コマンド: `grep -r "mex_kf_core" kalman/`

### 重複ファイルの確認

以下のファイルは類似機能を持つ可能性があります。
違いを確認し、統合または削除を検討してください。

1. **mex_eskf_do_update.cpp** vs **mex_eskf_do_cpp_update.cpp**
   - 両方とも`mex_meukf_step_v2`を呼び出し
   - 違いを確認して統合を検討

2. **mex_eskf_sensor_update.cpp** vs **mex_eskf_sensor_update_full.cpp**
   - 両方とも`mex_sensor_preprocessor`を使用
   - 違いを確認して統合を検討

3. **mex_ukf_update.cpp** vs **mex_ukf_update_minimal.cpp**
   - 両方ともUKF更新を実装
   - 違いを確認して統合を検討

## 段階的整理プラン

### Phase 1: クリーンアップ（即座に実行）

1. ✅ 古いバイナリの削除
2. ✅ `mex_quaternion_lib.cpp`をビルド対象に追加
3. ✅ 使用されていないバイナリの削除

### Phase 2: 調査（1週間以内）

1. 使用状況の確認
   - 上記の「調査が必要なファイル」を検索
   - 使用されていないファイルを特定

2. 重複ファイルの比較
   - 類似ファイルの機能比較
   - 統合可能性の評価

### Phase 3: 統合・削除（調査後）

1. 使用されていないファイルの削除
2. 重複ファイルの統合
3. ビルドスクリプトの整理

### Phase 4: ドキュメント更新

1. README.mdの更新
2. 各ファイルのコメント更新
3. APIドキュメントの作成

## ファイル整理の優先順位

### 高優先度（即座に対応）

1. **mex_quaternion_lib.cpp** - 使用されているがビルドされていない
2. **古いバイナリの削除** - 混乱の原因

### 中優先度（1週間以内）

1. **重複ファイルの確認** - `mex_eskf_do_update` vs `mex_eskf_do_cpp_update`
2. **使用状況不明ファイルの調査** - `mex_kalman_compute`, `mex_common_lib`等

### 低優先度（時間があるとき）

1. **APIの統一** - ハンドルベース vs 構造体ベース
2. **依存関係の最適化** - MEX間呼び出しの削減

## 推奨されるフォルダ構造

現在の構造は適切ですが、以下のサブフォルダを検討：

```
MEX/
├── ESKF/          # ESKF関連ファイル
├── Filters/       # フィルタコア（KF, EKF, UKF, MEUKF）
├── Utils/         # ユーティリティ
└── archive/       # 古いファイル（既存）
```

ただし、現在のフラット構造も問題ないため、無理に変更する必要はありません。

## 注意事項

1. **削除前に必ずバックアップ**
2. **使用状況を十分に確認してから削除**
3. **ビルドスクリプトの変更は慎重に**
4. **段階的に実行し、各段階でテスト**

## 次のステップ

1. このドキュメントをレビュー
2. Phase 1のクリーンアップを実行
3. Phase 2の調査を開始
4. 調査結果に基づいてPhase 3を実行



