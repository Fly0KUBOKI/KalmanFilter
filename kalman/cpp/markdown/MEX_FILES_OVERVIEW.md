# MEXファイル全体概要

## 調査日
2025年12月29日

## 概要
MEXフォルダには、MATLABからC++実装を呼び出すためのラッパー関数が含まれています。
すべての実装コードは`Inc/`、`Src/`、`Lib/`にあり、MEXファイルは型変換とインターフェース提供のみを行います。

## 統計情報

### ソースファイル数
- **合計**: 約40ファイル（.cpp）
- **ヘッダーファイル**: 1ファイル（mex_type_conv.hpp）

### バイナリファイル数
- **binフォルダ**: 40ファイル（.mexw64）
- **MEXフォルダ**: 3ファイル（.mexw64）※古いビルド残骸

### ビルド状況
- **ビルドスクリプトに含まれる**: 約30ファイル
- **ビルドスクリプトでスキップ**: 3ファイル
  - `mex_eskf_core.cpp` (legacy, locked)
  - `mex_quaternion_lib.cpp` (locked/skipped)
  - `mex_meukf_step.cpp` (mex_meukf_step_v2としてビルド)

## 主要カテゴリ

1. **ESKF関連** (約20ファイル)
   - コンストラクタ、初期化、状態管理
   - 予測、更新、後処理
   - センサー更新（accel, mag, gps, baro）

2. **フィルタコア** (約5ファイル)
   - KF, EKF, UKF, MEUKF
   - 統一フィルタインターフェース

3. **ユーティリティ** (約10ファイル)
   - 数学関数（クォータニオン、行列演算）
   - センサーフィルタ、前処理
   - 型変換ヘルパー

4. **完全実装** (約3ファイル)
   - `mex_run_eskf`: ESKF.mの完全置き換え
   - `mex_eskf_full`: 完全MEX化ESKF

## 依存関係の特徴

### MEX間の呼び出し
多くのMEXファイルが他のMEXファイルを`mexCallMATLAB`で呼び出しています。
これは段階的な移行戦略の結果です。

### 主要な呼び出しチェーン
```
mex_run_eskf
  ├─ mex_adaptive_predict
  ├─ mex_eskf_predict_postprocess
  ├─ mex_eskf_sensor_updates_full
  │   ├─ mex_sensor_preprocessor
  │   └─ mex_eskf_do_update
  │       ├─ mex_sensor_filter
  │       ├─ mex_meukf_step_v2
  │       └─ mex_eskf_update_postprocess
  ├─ mex_eskf_zupt
  ├─ mex_filter_management
  └─ mex_eskf_constructor
```

## 次のステップ

詳細は以下のドキュメントを参照：
- `MEX_FILES_CATEGORIES.md`: カテゴリ別詳細分類
- `MEX_FILES_DEPENDENCIES.md`: 依存関係マップ
- `MEX_FILES_MISSING_SOURCES.md`: ソースがないバイナリの調査
- `MEX_FILES_BUILD_STATUS.md`: ビルド状況と推奨事項


