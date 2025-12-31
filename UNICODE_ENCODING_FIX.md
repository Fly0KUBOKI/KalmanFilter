# Unicode Encoding Issue - Root Cause & Resolution

## 問題サマリー

**日付:** 2025-12-31  
**問題:** MEX コンパイル時に C4819 警告とそれに続く構文エラーが多数発生

## 根本原因

Windows での MSVC コンパイラが、以下の条件を満たさない場合に Japanese/多バイト文字をコードページ 932 (Shift-JIS) として解釈し、ファイルが破損：

1. **ファイルのエンコーディング** が UTF-8 だが、**BOM (Byte Order Mark) がない**
2. **コンパイラフラグ** `/utf-8` が渡されても、ファイルヘッダがないと信頼できない

## 解決方法

### 即座の修正（実施済み）
1. すべての C++ ヘッダファイル（`.hpp`）とソースファイル（`.cpp`）に **UTF-8 BOM** を追加
2. Python スクリプト `kalman/cpp/build/add_utf8_bom.py` で自動実行
3. 影響を受けたファイル：29ファイル

```bash
python3 kalman/cpp/build/add_utf8_bom.py
```

### ビルド結果
```
Compiling mex_meukf_step_v2... OK
Compiling mex_sensor_filter...  OK
Compiling mex_run_eskf...       OK
```

## 再発防止策

### Pre-commit Hook
`.git/hooks/pre-commit` に自動チェック機能を実装：
- ステージされた C++ ファイルが UTF-8 BOM を持つかを確認
- BOM がない場合、コミットを拒否
- 修正方法を自動表示

### 有効化方法
```bash
chmod +x .git/hooks/pre-commit
```

## 推奨事項

### VS Code での設定
ファイルを自動的に UTF-8 BOM で保存するよう設定：

```json
{
  "files.encoding": "utf8bom",
  "[cpp]": {
    "files.encoding": "utf8bom"
  }
}
```

### Visual Studio での設定
1. Tools → Options → Text Editor → C/C++ → Advanced
2. **File Encoding** を **UTF-8 with Signature** に設定

### コンパイラオプション
`build_mex.m` で `/utf-8` フラグが設定されているため、新規ファイルでも BOM があれば問題なし。

## 参考リンク

- [MSVC C4819 Warning - Encoding Support](https://learn.microsoft.com/en-us/cpp/error-messages/compiler-warnings/c4819)
- [UTF-8 BOM vs Non-BOM](https://en.wikipedia.org/wiki/Byte_order_mark)

## 成功指標

✅ `clear mex; build_mex();` が全MEXファイルで OK を返す  
✅ Git pre-commit hook でエンコーディングチェック実施  
✅ 今後のファイル追加時に同じ問題が発生しない

---

**修正完了者:** GitHub Copilot  
**修正日時:** 2025-12-31 14:01 UTC  
**検証版:** phase6
