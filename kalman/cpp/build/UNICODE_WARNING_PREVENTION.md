# Unicode警告（C4819）の再発防止ガイド

## 問題の説明

Visual Studioコンパイラ（MSVC）は、ソースファイルにUTF-8文字（日本語コメントなど）が含まれている場合、デフォルトでShift-JIS（コードページ932）として解釈しようとします。これにより、以下の警告が発生します：

```
warning C4819: ファイルは、現在のコード ページ (932) で表示できない文字を含んでいます。
データの損失を防ぐために、ファイルを Unicode 形式で保存してください。
```

## 解決策

### 1. 環境変数でUTF-8を指定（推奨・必須）

`build_mex.m`で複数の環境変数に`/utf-8`フラグを設定しています：

```matlab
setenv('COMPFLAGS', '/utf-8');   % C/C++共通
setenv('CXXFLAGS', '/utf-8');    % C++専用（優先度が高い）
setenv('CFLAGS', '/utf-8');      % C専用（念のため）
```

**重要**: MATLABの`mex`コマンドでは、コンパイラ固有のフラグ（`/utf-8`など）は`compile_opts`に直接追加できません。環境変数を使用する必要があります。

**なぜ複数の環境変数を設定するのか？**
- `COMPFLAGS`: C/C++共通のフラグ
- `CXXFLAGS`: C++専用のフラグ（優先度が高い）
- `CFLAGS`: C専用のフラグ

すべてに設定することで、MATLABの`mex`コマンドがどの環境変数を使用しても、確実に`/utf-8`フラグがコンパイラに渡されます。

このフラグにより：
- ソースファイルの文字セットがUTF-8として解釈される
- 実行時文字セットもUTF-8として扱われる
- Unicode警告（C4819）が抑制される

### 3. ファイルの保存形式

**重要**: ソースファイルはUTF-8（BOMなしまたはBOM付き）で保存してください。

- **推奨**: UTF-8 without BOM
- **代替**: UTF-8 with BOM

エディタの設定で確認：
- Visual Studio Code: 右下のエンコーディング表示を確認
- Visual Studio: ファイル > 高度な保存オプション > エンコーディング

## 再発防止チェックリスト

### 新しいファイルを作成する場合

1. ✅ ファイルをUTF-8（BOMなし）で保存
2. ✅ 日本語コメントを含む場合は、`/utf-8`フラグが有効であることを確認
3. ✅ ビルド時にUnicode警告が出ないことを確認

### 既存ファイルを編集する場合

1. ✅ ファイルのエンコーディングを確認（UTF-8であること）
2. ✅ 日本語コメントを追加した場合は、ビルドして警告が出ないことを確認
3. ✅ 警告が出た場合は、`build_mex.m`の`compile_opts`に`/utf-8`が含まれていることを確認

### ビルドスクリプトを変更する場合

1. ✅ `COMPFLAGS`環境変数に`/utf-8`フラグが設定されていることを確認
2. ✅ Windows環境（`ispc`）でのみ適用されることを確認
3. ✅ `compile_opts`には`/utf-8`を追加しない（環境変数のみ使用）
4. ✅ 他のコンパイラフラグと競合しないことを確認

## トラブルシューティング

### 警告が消えない場合

1. **環境変数が正しく設定されているか確認**
   ```matlab
   % MATLABで確認
   getenv('COMPFLAGS')
   getenv('CXXFLAGS')
   getenv('CFLAGS')
   % すべての出力に '/utf-8' が含まれていることを確認
   ```

2. **`mex`コマンドの詳細出力を確認**
   ```matlab
   % build_mex.mで'-v'オプションを追加して、実際に渡されているフラグを確認
   % 例: mex_args = [compile_opts, {'-v'}, inc_args, ...];
   ```

3. **コンパイラオプションファイルを直接編集**
   - MATLABのコンパイラオプションファイル（`mex_C++_win64.xml`など）を編集
   - ファイルの場所: `matlabroot/bin/win64/mexopts/`
   - `<COMPFLAGS>`セクションに`/utf-8`を追加

2. **ファイルのエンコーディングを確認**
   - ファイルを開いて、エディタのエンコーディング表示を確認
   - UTF-8以外の場合は、UTF-8に変換して保存

3. **環境変数の確認**
   ```matlab
   % MATLABで確認
   getenv('COMPFLAGS')
   ```

### コンパイルエラーが発生する場合

`/utf-8`フラグはVisual Studio 2015以降でサポートされています。古いバージョンを使用している場合は、以下の代替方法を検討：

1. ファイルをUTF-8 with BOMで保存
2. または、日本語コメントを英語に変更

## 参考資料

- [MSVC: `/utf-8` コンパイラオプション](https://learn.microsoft.com/en-us/cpp/build/reference/utf-8-set-source-and-executable-character-sets-to-utf-8)
- [C4819警告の詳細](https://learn.microsoft.com/en-us/cpp/error-messages/compiler-warnings/compiler-warning-level-1-c4819)

## 更新履歴

- 2025-12-31: 初版作成
  - `build_mex.m`に`/utf-8`フラグを追加
  - 再発防止チェックリストを追加

