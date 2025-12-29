# MEXフォルダ ドキュメント索引

このドキュメントは、`kalman/cpp/MEX/` フォルダ内のすべてのドキュメントファイルの索引です。

## 主要ドキュメント

### 📋 [README.md](README.md)
MEXフォルダの基本方針と設計原則
- MEXラッパーの役割と制約
- 実装コードの配置場所
- 正しい実装例と間違った実装例

### ✅ [SOURCE_CODE_STATUS.md](SOURCE_CODE_STATUS.md)
**最新のソースコード存在状況調査結果**
- すべてのMEXファイルのソースコード存在確認
- バイナリファイルとの対応関係
- ファイルサイズとコード量の統計
- 依存関係の確認

## 分析ドキュメント

### 🔍 [BINARY_SOURCE_COMPARISON.md](BINARY_SOURCE_COMPARISON.md)
バイナリとソースファイルの比較分析
- binディレクトリのバイナリ一覧
- MEXディレクトリのソースファイル一覧
- バイナリはあるがソースがないファイル（過去の状況）
- ソースはあるがバイナリがないファイル

**注意**: このドキュメントは過去の状況を記録したものです。現在はすべてのソースコードが存在します（[SOURCE_CODE_STATUS.md](SOURCE_CODE_STATUS.md)を参照）。

### 🔗 [DEPENDENCY_ANALYSIS.md](DEPENDENCY_ANALYSIS.md)
MEXファイル間の依存関係分析
- `mexCallMATLAB`による依存関係
- 依存関係ツリー
- ビルド順序の推奨

### 📦 [UNUSED_FILES_ANALYSIS.md](UNUSED_FILES_ANALYSIS.md)
未使用ファイルの分析
- コンパイルされていないファイルのリスト
- 使用状況の確認方法
- 整理推奨事項

## 復元・復旧ドキュメント

### 🔧 [GIT_RESTORE_COMMANDS.md](GIT_RESTORE_COMMANDS.md)
Gitでソースコードを復元するコマンド集
- ファイル履歴の確認方法
- ファイル復元の手順
- 一括復元スクリプト（Bash/PowerShell）

**注意**: 現在はすべてのソースコードが存在するため、このドキュメントは参考用です。

## ドキュメントの更新状況

| ドキュメント | 最終更新 | 状態 |
|------------|---------|------|
| README.md | - | ✅ 最新 |
| SOURCE_CODE_STATUS.md | 2025-01-29 | ✅ 最新 |
| BINARY_SOURCE_COMPARISON.md | - | ⚠️ 過去の状況（参考用） |
| DEPENDENCY_ANALYSIS.md | - | ✅ 有効 |
| UNUSED_FILES_ANALYSIS.md | - | ✅ 有効 |
| GIT_RESTORE_COMMANDS.md | - | ⚠️ 参考用 |

## ドキュメントの使い分け

### ソースコードの存在確認
→ **[SOURCE_CODE_STATUS.md](SOURCE_CODE_STATUS.md)** を参照

### 依存関係の理解
→ **[DEPENDENCY_ANALYSIS.md](DEPENDENCY_ANALYSIS.md)** を参照

### ビルド順序の確認
→ **[DEPENDENCY_ANALYSIS.md](DEPENDENCY_ANALYSIS.md)** の「ビルド順序の推奨」セクションを参照

### 未使用ファイルの整理
→ **[UNUSED_FILES_ANALYSIS.md](UNUSED_FILES_ANALYSIS.md)** を参照

### 過去の状況の確認
→ **[BINARY_SOURCE_COMPARISON.md](BINARY_SOURCE_COMPARISON.md)** を参照（参考用）

## 関連ファイル

- `RESTORE_FIXED.ps1`: PowerShellスクリプト（ファイル復元用）
- `zupt.m.txt`: ZUPT関連のメモファイル

