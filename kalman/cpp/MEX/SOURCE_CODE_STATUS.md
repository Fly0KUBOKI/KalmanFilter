# MEXフォルダ ソースコード存在状況調査

**調査日**: 2025年1月29日  
**調査対象**: `kalman/cpp/MEX/` フォルダ内のすべてのC++ソースファイル

## 調査結果サマリー

✅ **すべてのMEXファイルに対応するソースコードが存在しています**

- **調査対象ファイル数**: 13個
- **ソースコード存在**: 13個（100%）
- **ソースコード欠落**: 0個

## 詳細調査結果

### 1. ソースコードが存在するファイル（13個）

| # | ファイル名 | 行数 | 状態 | 備考 |
|---|-----------|------|------|------|
| 1 | `mex_adaptive_predict.cpp` | 137行 | ✅ 存在 | Phase 4 scaffolding実装 |
| 2 | `mex_eskf_constructor.cpp` | 491行 | ✅ 存在 | Phase 1: ESKFコンストラクタ |
| 3 | `mex_eskf_do_update.cpp` | 282行 | ✅ 存在 | do_cpp_update()完全MEX化 |
| 4 | `mex_eskf_predict_postprocess.cpp` | 279行 | ✅ 存在 | predict()後処理ラッパー |
| 5 | `mex_eskf_sensor_updates_full.cpp` | 466行 | ✅ 存在 | sensor_updates()全体MEX化 |
| 6 | `mex_eskf_update_postprocess.cpp` | 265行 | ✅ 存在 | do_cpp_update()後処理ラッパー |
| 7 | `mex_eskf_zupt.cpp` | 252行 | ✅ 存在 | ZUPT (Zero Velocity Update) |
| 8 | `mex_filter_management.cpp` | 98行 | ✅ 存在 | Phase 5 フィルタ管理ユーティリティ |
| 9 | `mex_meukf_step.cpp` | 275行 | ✅ 存在 | MEUKFステップ実装 |
| 10 | `mex_quaternion_lib.cpp` | 298行 | ✅ 存在 | クォータニオン演算ライブラリ |
| 11 | `mex_run_eskf.cpp` | 595行 | ✅ 存在 | 完全ESKF実装（単一MEX） |
| 12 | `mex_sensor_filter.cpp` | 307行 | ✅ 存在 | センサーフィルタライブラリ |
| 13 | `mex_sensor_preprocessor.cpp` | 122行 | ✅ 存在 | Phase3 センサー前処理 |

### 2. 補助ファイル

| ファイル名 | 状態 | 説明 |
|-----------|------|------|
| `mex_type_conv.hpp` | ✅ 存在 | MATLAB型変換ヘルパー |

## バイナリファイルとの対応関係

`kalman/cpp/bin/` ディレクトリに存在するバイナリファイル（.mexw64）とソースファイルの対応：

| バイナリファイル | ソースファイル | 状態 |
|----------------|--------------|------|
| `mex_adaptive_predict.mexw64` | `mex_adaptive_predict.cpp` | ✅ 対応 |
| `mex_eskf_constructor.mexw64` | `mex_eskf_constructor.cpp` | ✅ 対応 |
| `mex_eskf_do_update.mexw64` | `mex_eskf_do_update.cpp` | ✅ 対応 |
| `mex_eskf_predict_postprocess.mexw64` | `mex_eskf_predict_postprocess.cpp` | ✅ 対応 |
| `mex_eskf_sensor_updates_full.mexw64` | `mex_eskf_sensor_updates_full.cpp` | ✅ 対応 |
| `mex_eskf_update_postprocess.mexw64` | `mex_eskf_update_postprocess.cpp` | ✅ 対応 |
| `mex_eskf_zupt.mexw64` | `mex_eskf_zupt.cpp` | ✅ 対応 |
| `mex_filter_management.mexw64` | `mex_filter_management.cpp` | ✅ 対応 |
| `mex_meukf_step_v2.mexw64` | `mex_meukf_step.cpp` | ✅ 対応（別名でビルド） |
| `mex_quaternion_lib.mexw64` | `mex_quaternion_lib.cpp` | ✅ 対応 |
| `mex_run_eskf.mexw64` | `mex_run_eskf.cpp` | ✅ 対応 |
| `mex_sensor_filter.mexw64` | `mex_sensor_filter.cpp` | ✅ 対応 |
| `mex_sensor_preprocessor.mexw64` | `mex_sensor_preprocessor.cpp` | ✅ 対応 |

**結論**: すべてのバイナリファイルに対応するソースコードが存在しています。

## ファイルサイズとコード量

| ファイル名 | 行数 | 推定サイズ | 主要機能 |
|-----------|------|-----------|---------|
| `mex_run_eskf.cpp` | 595行 | 最大 | 完全ESKF実装 |
| `mex_eskf_sensor_updates_full.cpp` | 466行 | 大 | センサー更新統合 |
| `mex_eskf_constructor.cpp` | 491行 | 大 | 初期化処理 |
| `mex_sensor_filter.cpp` | 307行 | 中 | センサーフィルタ |
| `mex_quaternion_lib.cpp` | 298行 | 中 | クォータニオン演算 |
| `mex_eskf_do_update.cpp` | 282行 | 中 | 更新処理 |
| `mex_eskf_predict_postprocess.cpp` | 279行 | 中 | 予測後処理 |
| `mex_meukf_step.cpp` | 275行 | 中 | MEUKFステップ |
| `mex_eskf_update_postprocess.cpp` | 265行 | 中 | 更新後処理 |
| `mex_eskf_zupt.cpp` | 252行 | 中 | ZUPT処理 |
| `mex_adaptive_predict.cpp` | 137行 | 小 | 適応予測 |
| `mex_sensor_preprocessor.cpp` | 122行 | 小 | 前処理 |
| `mex_filter_management.cpp` | 98行 | 小 | フィルタ管理 |

**合計**: 約3,727行のC++コード

## 依存関係の確認

各ファイルの依存関係（`mexCallMATLAB`使用による）：

### 主要な依存関係

1. **mex_run_eskf.cpp**
   - 依存: `mex_adaptive_predict`, `mex_eskf_predict_postprocess`, `mex_eskf_sensor_updates_full`, `mex_filter_management`, `mex_eskf_zupt`, `mex_eskf_constructor`

2. **mex_eskf_do_update.cpp**
   - 依存: `mex_sensor_filter`, `mex_meukf_step_v2`, `mex_eskf_update_postprocess`

3. **mex_eskf_sensor_updates_full.cpp**
   - 依存: `mex_sensor_preprocessor`, `mex_eskf_do_update`

4. **mex_eskf_predict_postprocess.cpp**
   - 依存: `mex_quaternion_lib`, `mex_sensor_filter`

5. **mex_eskf_update_postprocess.cpp**
   - 依存: `mex_sensor_filter`

## 結論

✅ **すべてのMEXファイルのソースコードが完全に存在しています**

- ソースコードの欠落はありません
- すべてのバイナリファイルに対応するソースコードが確認されました
- コードは適切に構造化され、依存関係も明確です

## 推奨事項

1. **定期的なバックアップ**: ソースコードは完全に存在していますが、定期的なバックアップを推奨します
2. **バージョン管理**: Gitでの管理を継続し、重要な変更はコミットしてください
3. **ドキュメント更新**: コード変更時は関連ドキュメントも更新してください

