# テスト完了後に削除可能なファイル一覧

## 更新日時
2025年12月31日

## 削除可能なファイル

### ⚠️ 重要: テスト完了後に削除してください

以下のファイルは`mex_run_eskf`に統合済みのため、削除可能です。

---

## 1. ソースファイル（削除推奨）

### `kalman/cpp/MEX/mex_meukf_step.cpp`
- **理由**: `mex_meukf_step_v2`の機能が`mex_run_eskf_sensor_updates.hpp`に統合済み
- **統合先**: `kalman/cpp/Inc/MEX/mex_run_eskf_sensor_updates.hpp`
- **統合内容**: `MEUKFCore::step()`を直接呼び出し

### `kalman/cpp/MEX/mex_sensor_filter.cpp`
- **理由**: `mex_sensor_filter`の機能が`mex_run_eskf_impl.hpp`に統合済み
- **統合先**: `kalman/cpp/Inc/MEX/mex_run_eskf_impl.hpp`
- **統合内容**: `g_filter_lib.reset_all_zero()`を`do_init()`内で自動実行

---

## 2. バイナリファイル（削除推奨）

### `kalman/cpp/bin/mex_meukf_step_v2.mexw64`
- **理由**: `mex_run_eskf.mexw64`に統合済み
- **削除タイミング**: テスト完了後

### `kalman/cpp/bin/mex_sensor_filter.mexw64`
- **理由**: `mex_run_eskf.mexw64`に統合済み
- **削除タイミング**: テスト完了後

---

## 3. 削除コマンド（テスト完了後）

### Windows (PowerShell)

```powershell
# ソースファイルを削除
Remove-Item "kalman\cpp\MEX\mex_meukf_step.cpp"
Remove-Item "kalman\cpp\MEX\mex_sensor_filter.cpp"

# バイナリファイルを削除
Remove-Item "kalman\cpp\bin\mex_meukf_step_v2.mexw64"
Remove-Item "kalman\cpp\bin\mex_sensor_filter.mexw64"
```

### Linux/Mac

```bash
# ソースファイルを削除
rm kalman/cpp/MEX/mex_meukf_step.cpp
rm kalman/cpp/MEX/mex_sensor_filter.cpp

# バイナリファイルを削除
rm kalman/cpp/bin/mex_meukf_step_v2.mexw64
rm kalman/cpp/bin/mex_sensor_filter.mexw64
```

---

## 4. 既に更新済みのファイル

以下のファイルは既に更新済みです（削除不要）：

### ✅ `kalman/cpp/build/build_mex.m`
- **変更内容**: `mex_meukf_step.cpp`と`mex_sensor_filter.cpp`のビルドエントリをコメントアウト
- **行番号**: 175-183行目

### ✅ `kalman/run_batch_10sets.m`
- **変更内容**: `mex_sensor_filter`の呼び出しをコメントアウト
- **行番号**: 28-35行目

---

## 5. 削除前の確認事項

### 必須チェックリスト

- [ ] `run_batch_10sets()`が正常に実行できる
- [ ] 10/10 PASS (100%)を確認
- [ ] 推定精度が既存と同等であることを確認
- [ ] NaN/Infが発生しないことを確認
- [ ] `mex_run_eskf.mexw64`が正常にビルドできることを確認

### 削除後の確認

- [ ] `mex_run_eskf`が正常に動作することを確認
- [ ] ビルドスクリプトが正常に動作することを確認

---

## 6. 統合状況の確認

統合が完了していることを確認するには、以下のファイルを参照：

- [INTEGRATION_STATUS_AND_CLEANUP.md](INTEGRATION_STATUS_AND_CLEANUP.md) - 統合状況の詳細
- [MEX_INTEGRATION_COMPLETE.md](MEX_INTEGRATION_COMPLETE.md) - 統合完了レポート

---

## 7. 注意事項

### バックアップの推奨

削除前に、念のためバックアップを取ることを推奨します：

```bash
# バックアップディレクトリを作成
mkdir -p backup/mex_files

# ファイルをコピー
cp kalman/cpp/MEX/mex_meukf_step.cpp backup/mex_files/
cp kalman/cpp/MEX/mex_sensor_filter.cpp backup/mex_files/
cp kalman/cpp/bin/mex_meukf_step_v2.mexw64 backup/mex_files/
cp kalman/cpp/bin/mex_sensor_filter.mexw64 backup/mex_files/
```

### Git管理

Gitを使用している場合、削除前にコミットを推奨：

```bash
git add -A
git commit -m "Remove integrated MEX files (mex_meukf_step_v2, mex_sensor_filter)"
```

---

## 8. 削除後の状態

### 統合前
- **MEXファイル数**: 3つ
  - `mex_run_eskf.mexw64`
  - `mex_meukf_step_v2.mexw64`
  - `mex_sensor_filter.mexw64`

### 統合後（削除後）
- **MEXファイル数**: 1つ
  - `mex_run_eskf.mexw64` ⭐

---

## 参考資料

- [INTEGRATION_STATUS_AND_CLEANUP.md](INTEGRATION_STATUS_AND_CLEANUP.md) - 統合状況とクリーンアップガイド
- [MEX_INTEGRATION_COMPLETE.md](MEX_INTEGRATION_COMPLETE.md) - 統合完了レポート

