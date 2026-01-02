# MEXファイル移動計画

## 対象ファイル

以下のファイルを`Inc/ESKF/`と`Src/ESKF/`から`Inc/MEX/`と`MEX/`に移動します：

### 移動対象

1. **`Inc/ESKF/eskf_initializer.hpp`** → **`Inc/MEX/mex_eskf_initializer.hpp`**
   - 理由: `mex.h`を無条件でインクルード、`initialize_eskf_from_matlab()`関数がMEX専用

2. **`Src/ESKF/eskf_initializer.cpp`** → **`MEX/mex_eskf_initializer.cpp`**
   - 理由: MATLAB API (`mxArray`, `mexErrMsgIdAndTxt`など) を直接使用、MEX専用の実装

### 移動対象外（検討済み）

- **`Src/ESKF/eskf_sensor_updates.cpp`**: `mxIsNaN`のみ使用、純粋なC++実装の一部としても機能。条件付きコンパイルで対応可能だが、今回は移動対象外

## 手順

1. ファイルを移動
2. 名前空間の調整（必要に応じて）
3. インクルードパスの更新
4. ビルドスクリプトの更新
5. 動作確認

