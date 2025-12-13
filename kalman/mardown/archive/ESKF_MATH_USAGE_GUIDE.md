````markdown
# eskf_math ライブラリ使用ガイド

## 概要
`eskf_math`は、センサーや状態量に依存しない純粋な数学計算を提供するC++/MEXライブラリです。

## 設計思想
- **入力→出力のみ**: すべての関数は入力行列を受け取り、出力行列を返す
- **ステートレス**: 内部状態を持たず、同じ入力には常に同じ出力
- **センサー非依存**: 特定のセンサー種別に依存しない汎用計算
- **math.hスタイル**: C標準ライブラリの`sin()`, `cos()`のような使用感

## インストール

### 1. MEXファイルのビルド
```matlab
cd cpp/build
build_mex
```

### 2. パスの追加
```matlab
addpath('Common/Math')
addpath('cpp/bin')
```

## 使用方法

### 基本的な呼び出し
```matlab
output = eskf_math('function_name', input1, input2, ...);
```

## 関数リファレンス

### クォータニオン操作

#### quaternion_integration
角速度からクォータニオンを積分

**入力:**
- `q` (4×1): 現在のクォータニオン [w; x; y; z]
- `w` (3×1): 角速度 [rad/s]
- `dt` (scalar): 時間ステップ [s]

**出力:**
- `q_new` (4×1): 積分後のクォータニオン

**例:**
```matlab
q = [1; 0; 0; 0];  % 初期姿勢
w = [0; 0; 0.1];   % Z軸周りに0.1 rad/s
dt = 0.01;
q_new = eskf_math('quaternion_integration', q, w, dt);
```

---

(省略: 長文の内容はアーカイブに保存されています)

```"