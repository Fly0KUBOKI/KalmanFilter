# 統合状況とクリーンアップガイド

## 更新日時
2025年12月31日

## 現在の統合状況

### ✅ 統合完了済み

#### 1. `mex_meukf_step_v2` → `mex_run_eskf`に統合 ✅

**統合場所:**
- `kalman/cpp/Inc/MEX/mex_run_eskf_sensor_updates.hpp`
  - `matlab_to_meukf_input()` - MATLAB構造体 → MEUKFInput変換
  - `meukf_output_to_matlab()` - MEUKFOutput → MATLAB構造体変換
  - `MEUKFCore::step()`を直接呼び出し（`mexCallMATLAB`を削除）

**変更内容:**
- `mexCallMATLAB`経由の呼び出しを削除
- C++直接呼び出しに変更
- 型変換ロジックを統合

#### 2. `mex_sensor_filter` → `mex_run_eskf`に統合 ✅

**統合場所:**
- `kalman/cpp/Inc/MEX/mex_run_eskf_impl.hpp`
  - `do_init()`内で`g_filter_lib.reset_all_zero()`を自動実行

**変更内容:**
- `mex_run_eskf('init')`内でセンサーフィルターライブラリを自動初期化
- MATLABコードからの直接呼び出しを不要に

---

## 現在のMEXファイル構成

### 必須MEXファイル（1つ）

1. **`mex_run_eskf.mexw64`** ⭐ **唯一の必須MEXファイル**
   - すべての機能が統合済み
   - `mex_meukf_step_v2`と`mex_sensor_filter`の機能を含む

### 削除可能なMEXファイル（2つ）

1. **`mex_meukf_step_v2.mexw64`** - 統合済み
2. **`mex_sensor_filter.mexw64`** - 統合済み

---

## 削除可能なファイル一覧

### 1. ソースファイル（削除推奨）

以下のソースファイルは統合済みのため、削除可能です：

```
kalman/cpp/MEX/mex_meukf_step.cpp
kalman/cpp/MEX/mex_sensor_filter.cpp
```

**注意**: テスト完了後に削除を推奨

### 2. バイナリファイル（削除推奨）

以下のバイナリファイルは統合済みのため、削除可能です：

```
kalman/cpp/bin/mex_meukf_step_v2.mexw64
kalman/cpp/bin/mex_sensor_filter.mexw64
```

**注意**: テスト完了後に削除を推奨

### 3. ビルドスクリプトのエントリ（コメントアウト推奨）

`kalman/cpp/build/build_mex.m`の以下の行をコメントアウト：

```matlab
% 175-179行目: mex_meukf_step.cpp
% 181-183行目: mex_sensor_filter.cpp
```

### 4. MATLABコードの呼び出し（既にコメントアウト済み）

`kalman/run_batch_10sets.m`の28-35行目：
- 既にコメントアウトされているか確認
- 統合により不要になったため、削除可能

---

## クリーンアップ手順

### ステップ1: テスト実行（必須）

統合が正しく動作することを確認：

```matlab
cd kalman
clear mex
run_batch_10sets()
```

**期待される結果:**
- 10/10 PASS (100%)
- 既存のテスト結果と同等の精度
- NaN/Infが発生しない

### ステップ2: ビルドスクリプトの更新

`build_mex.m`から以下のエントリをコメントアウト：

```matlab
% meukf_core_cpp = fullfile(src_dir, 'MEUKF', 'meukf_core.cpp');
% if exist('mex_meukf_step.cpp', 'file') && exist(meukf_core_cpp, 'file')
%     if wants('mex_meukf_step') && build_single_mex('mex_meukf_step.cpp', compile_opts, inc_args, {meukf_core_cpp}, bin_dir, 'mex_meukf_step_v2', log_fid)
%         built_count = built_count + 1;
%     end
% end

% if wants('mex_sensor_filter') && build_single_mex('mex_sensor_filter.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
%     built_count = built_count + 1;
% end
```

### ステップ3: ソースファイルの削除（テスト完了後）

テストが成功したら、以下のファイルを削除：

```bash
# ソースファイル
kalman/cpp/MEX/mex_meukf_step.cpp
kalman/cpp/MEX/mex_sensor_filter.cpp

# バイナリファイル
kalman/cpp/bin/mex_meukf_step_v2.mexw64
kalman/cpp/bin/mex_sensor_filter.mexw64
```

### ステップ4: MATLABコードのクリーンアップ（オプション）

`run_batch_10sets.m`の28-35行目を完全に削除：

```matlab
% 以下のコードを削除（既にコメントアウト済みの場合）
% if exist('mex_sensor_filter','file') == 3
%     try
%         mex_sensor_filter('reset_zero');
%     catch
%         try mex_sensor_filter('reset'); catch, end
%     end
% end
```

---

## 統合前後の比較

### 統合前

```
MATLABコード
├─ mex_run_eskf (直接呼び出し)
│  └─ mexCallMATLAB → mex_meukf_step_v2
└─ mex_sensor_filter (直接呼び出し - 初期化)
```

**MEXファイル数**: 3つ

### 統合後

```
MATLABコード
└─ mex_run_eskf (直接呼び出し)
   ├─ MEUKFCore::step() (C++直接呼び出し)
   └─ g_filter_lib.reset_all_zero() (C++直接呼び出し)
```

**MEXファイル数**: 1つ

---

## 統合のメリット

### 1. パフォーマンス向上
- ✅ `mexCallMATLAB`のオーバーヘッドを削減
- ✅ 型変換の最適化
- ✅ メモリコピーの削減

### 2. コードの簡素化
- ✅ MEXファイル数が3つ → 1つに削減
- ✅ 依存関係の簡素化
- ✅ ビルド時間の短縮

### 3. 保守性の向上
- ✅ コードが1箇所に集約
- ✅ デバッグが容易
- ✅ テストが簡素化

---

## 注意事項

### 互換性の維持

統合後も既存のMATLABコードとの互換性を保つため、以下の処理を実装：

1. **`innov`フィールド**: `dbg_out`に追加（既存コードが期待）
2. **`dx`フィールド**: `dbg_out`に追加（既存コードが期待）
3. **`input_update_gps`と`input_noise_gps`**: `dbg_out`に追加（デバッグ用）

### テストの重要性

統合後は必ず以下を確認：
- ✅ すべてのテストがPASSすること
- ✅ 推定精度が既存と同等であること
- ✅ NaN/Infが発生しないこと

---

## 参考資料

- [MEX_INTEGRATION_COMPLETE.md](MEX_INTEGRATION_COMPLETE.md) - 統合完了レポート
- [kalman/cpp/markdown/MEX_INTEGRATION_ANALYSIS.md](kalman/cpp/markdown/MEX_INTEGRATION_ANALYSIS.md) - 統合分析
- [kalman/cpp/markdown/MEX_FILES_CURRENT_STATUS.md](kalman/cpp/markdown/MEX_FILES_CURRENT_STATUS.md) - 現在の状態

