# MEX部分と純粋なC++実装部分の分離分析

**作成日**: 2025-01-XX  
**目的**: MATLABインターフェース（MEX）と純粋なC++実装の明確な分離と依存関係の分析

---

## 1. 概要

コードベースは以下の2つの明確な部分に分離されています：

1. **MEX部分（MATLAB橋渡し）**: MATLABとC++実装を接続するインターフェース層
2. **純粋なC++実装部分**: MATLABに依存しない独立した実装

---

## 2. MEX部分（MATLAB橋渡し）

### 2.1 構成ファイル

#### MEX/配下（エントリーポイント）
```
MEX/
├── mex_run_eskf.cpp          # ESKF実装のMEXエントリーポイント
└── mex_meukf_step.cpp        # MEUKF実装のMEX（非推奨・deprecated）
```

#### Inc/MEX/配下（MEX用ヘッダー・ヘルパー）
```
Inc/MEX/
├── mex_eskf_common.hpp       # MEX用共通インクルード・定義
├── mex_run_eskf_impl.hpp     # mex_run_eskf用の実装関数群
├── mex_run_eskf_sensor_updates.hpp  # センサー更新処理
├── mex_run_eskf_filter_ops.hpp      # フィルター操作
├── mex_type_conversion.hpp   # MATLAB↔C++型変換関数
└── mex_helpers.hpp           # MEX用ヘルパー関数
```

### 2.2 MEX部分の役割

1. **MATLABデータ型 ↔ C++型の変換**
   - `mxArray` ↔ `float[]`, `double[]`, 構造体
   - 型チェックとバリデーション
   - 座標系変換（MATLABの列優先 ↔ C++の行優先）

2. **mexFunctionエントリーポイント**
   - MATLABから呼び出される関数のインターフェース
   - 引数の解析とバリデーション
   - エラーハンドリング

3. **純粋なC++実装の呼び出し**
   - `Inc/`, `Src/`, `Lib/`の関数・クラスを呼び出す
   - アルゴリズム実装は一切含まない

### 2.3 MEX部分が依存する純粋なC++実装

MEX部分は以下の純粋なC++実装に依存しています：

```
Inc/MEX/mex_eskf_common.hpp がインクルード:
├── Common/Math/fixed_matrix.hpp
├── Common/Math/vector_utils.hpp
├── Common/Math/quaternion_lib.hpp
├── Common/Math/statistics.hpp
├── ESKF/eskf_core.hpp
├── ESKF/eskf_postprocess.hpp
├── ESKF/eskf_state.hpp
├── ESKF/eskf_runner.hpp
├── ESKF/eskf_initializer.hpp
├── Common/filter_management.hpp
├── Common/Sensor/sensor_filter.hpp
├── Common/Sensor/sensor_preprocessor.hpp
└── ESKF/eskf_sensor_updates.hpp

Inc/MEX/mex_run_eskf_impl.hpp が追加でインクルード:
└── MEUKF/meukf_core.hpp
```

### 2.4 MEX部分の特徴

- ✅ **許可されるコード**:
  - MATLABデータ型 ↔ C++型の変換
  - `mexFunction`の実装
  - エラーチェックとバリデーション
  - 純粋なC++実装の呼び出し

- ❌ **禁止されるコード**:
  - アルゴリズム実装（カルマンフィルタ計算、行列演算など）
  - ビジネスロジック
  - 状態管理（グローバル状態は最小限）

---

## 3. 純粋なC++実装部分

### 3.1 構成ファイル

#### Inc/配下（ヘッダー - MEX/MEX/以外）
```
Inc/
├── Common/           # 共通ユーティリティ
│   ├── Math/        # 数学関数（行列、クォータニオンなど）
│   ├── Sensor/      # センサー処理
│   └── Validation/  # バリデーション
├── ESKF/            # Error State Kalman Filter
├── EKF/             # Extended Kalman Filter
├── UKF/             # Unscented Kalman Filter
├── KF/              # Kalman Filter
└── MEUKF/           # Modified Extended Unscented Kalman Filter
```

#### Src/配下（実装）
```
Src/
├── Common/          # 共通実装
├── ESKF/            # ESKF実装（12ファイル）
├── EKF/             # EKF実装（1ファイル）
├── UKF/             # UKF実装（1ファイル）
└── MEUKF/           # MEUKF実装（2ファイル）
```

#### Lib/配下（独立ライブラリ）
```
Lib/
├── Common/          # 共通定義（types.hpp）
├── Matrix/          # 行列ライブラリ
├── Quaternion/      # クォータニオンライブラリ
└── KalmanCore/      # カルマンフィルタ基盤
```

### 3.2 純粋なC++実装の特徴

- ✅ **MATLABに依存しない**: `mex.h`, `mxArray`などのMEX APIを一切使用しない
- ✅ **独立性**: 他のC++環境（Arduino、ROS、スタンドアロンC++など）でも使用可能
- ✅ **アルゴリズム実装**: 全てのカルマンフィルタアルゴリズムを含む
- ✅ **固定サイズ配列**: 動的メモリ確保を禁止
- ✅ **float型**: 浮動小数点は`float`型を使用

### 3.3 純粋なC++実装の依存関係

純粋なC++実装は以下の依存関係を持ちます：

```
各フィルター実装
├── Inc/Common/Math/*     # 数学関数
├── Inc/Common/Sensor/*   # センサー処理
├── Inc/Common/*          # 共通ユーティリティ
├── Lib/Matrix/*          # 行列ライブラリ
├── Lib/Quaternion/*      # クォータニオンライブラリ
└── Lib/KalmanCore/*      # カルマンフィルタ基盤
```

**重要な原則**: 純粋なC++実装は`Inc/MEX/`や`MEX/`に依存しない

---

## 4. 依存関係図

```
┌─────────────────────────────────────────────────────────────┐
│                    MATLAB層                                 │
│              (run_simulation.m, etc.)                        │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│              MEX部分（MATLAB橋渡し）                         │
│  ┌──────────────┐  ┌──────────────────────────────────┐    │
│  │ MEX/*.cpp    │  │ Inc/MEX/*.hpp                    │    │
│  │              │  │  - mex_eskf_common.hpp           │    │
│  │ - mex_run_   │  │  - mex_run_eskf_impl.hpp         │    │
│  │   eskf.cpp   │  │  - mex_type_conversion.hpp       │    │
│  │ - mex_meukf_ │  │  - mex_helpers.hpp               │    │
│  │   step.cpp   │  │  - mex_run_eskf_sensor_*.hpp     │    │
│  └──────────────┘  └──────────────────────────────────┘    │
└────────────────────┬────────────────────────────────────────┘
                     │ 依存（#include）
                     ▼
┌─────────────────────────────────────────────────────────────┐
│           純粋なC++実装部分（MATLAB非依存）                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                 │
│  │ Inc/     │  │ Src/     │  │ Lib/     │                 │
│  │ (Header) │  │ (Impl)   │  │ (Lib)    │                 │
│  │          │  │          │  │          │                 │
│  │ - ESKF/  │  │ - ESKF/  │  │ - Matrix/│                 │
│  │ - EKF/   │  │ - EKF/   │  │ - Quat/  │                 │
│  │ - UKF/   │  │ - UKF/   │  │ - Kalman │                 │
│  │ - MEUKF/ │  │ - MEUKF/ │  │   Core/  │                 │
│  │ - Common/│  │ - Common/│  │ - Common/│                 │
│  └──────────┘  └──────────┘  └──────────┘                 │
└─────────────────────────────────────────────────────────────┘
```

**依存方向**: MEX部分 → 純粋なC++実装（一方向）

---

## 5. 具体的な呼び出し例

### 5.1 MEXから純粋なC++実装への呼び出し

#### 例1: ESKFRunner::predictの呼び出し
```cpp
// Inc/MEX/mex_run_eskf_impl.hpp
inline void call_predict(ESKFState* s, const double* a_meas, const double* w_meas) {
    ESKFRunner::predict(s, a_meas, w_meas);  // ← 純粋なC++実装を呼び出し
}
```

#### 例2: MEUKFCore::stepの呼び出し
```cpp
// Inc/MEX/mex_run_eskf_impl.hpp
inline void do_meukf_step(...) {
    meukf::MEUKFCore::step(input, output);  // ← 純粋なC++実装を呼び出し
}
```

#### 例3: ESKFCore::update_zuptの呼び出し
```cpp
// Inc/MEX/mex_run_eskf_filter_ops.hpp
ESKFCore::update_zupt(v_in, P_in, v_out, P_out);  // ← 純粋なC++実装を呼び出し
```

### 5.2 型変換の例

```cpp
// Inc/MEX/mex_type_conversion.hpp
inline void mxArrayToFloatArray(const mxArray* arr, float* out, std::size_t n) {
    // MATLAB mxArray → C++ float配列
    const float* pf = (const float*)mxGetData(arr);
    for (std::size_t i = 0; i < n; ++i) out[i] = pf[i];
}
```

---

## 6. 分離の確認方法

### 6.1 MEX部分がMATLAB APIを使用していることの確認

```bash
# MEX部分でMATLAB APIが使用されていることを確認
grep -r "mex\.h\|mxArray\|mexErrMsgIdAndTxt\|mexFunction" kalman/cpp/MEX/
grep -r "mex\.h\|mxArray\|mexErrMsgIdAndTxt\|mexFunction" kalman/cpp/Inc/MEX/
```

**結果**: MEX部分のみがMATLAB APIを使用（期待通り）

### 6.2 純粋なC++実装がMATLAB APIを使用していないことの確認

```bash
# 純粋なC++実装でMATLAB APIが使用されていないことを確認
grep -r "mex\.h\|mxArray\|mexErrMsgIdAndTxt\|mexFunction" kalman/cpp/Inc/ --exclude-dir=MEX
grep -r "mex\.h\|mxArray\|mexErrMsgIdAndTxt\|mexFunction" kalman/cpp/src/
grep -r "mex\.h\|mxArray\|mexErrMsgIdAndTxt\|mexFunction" kalman/cpp/Lib/
```

**結果**: 
- `Inc/`（MEX/以外）: 2ファイルがMATLAB APIを参照
  - `eskf_initializer.hpp`: 無条件で`mex.h`をインクルード（改善が必要）
  - `sensor_filter.hpp`: 条件付きコンパイル（`#ifdef MATLAB_MEX_FILE`）で使用（問題なし）
- `Src/`: 2ファイルがMATLAB APIを使用（`eskf_initializer.cpp`, `eskf_sensor_updates.cpp`）
- `Lib/`: 0ファイル（完全に独立）

**詳細**:
- `eskf_initializer.hpp/cpp`: `initialize_eskf_from_matlab()`関数がMATLABの`mxArray`を受け取る。これはMEX専用の初期化関数なので、`Inc/MEX/`に移動するか、条件付きコンパイルで対応すべき。
- `sensor_filter.hpp`: `#ifdef MATLAB_MEX_FILE`で条件付きに`mex.h`をインクルードし、MEXビルド時のみ`mexPrintf`を使用。非MEXビルド時は`printf`を使用するため問題なし。

---

## 7. 統計情報

### 7.1 ファイル数

| カテゴリ | ディレクトリ | ファイル数 | 説明 |
|---------|------------|----------|------|
| **MEX部分** | `MEX/` | 2 | MEXエントリーポイント |
| **MEX部分** | `Inc/MEX/` | 6 | MEX用ヘッダー・ヘルパー |
| **純粋なC++** | `Inc/` (MEX/以外) | 36 | ヘッダーファイル |
| **純粋なC++** | `Src/` | 12 | 実装ファイル |
| **純粋なC++** | `Lib/` | 5 | 独立ライブラリ |

### 7.2 コード行数（概算）

- **MEX部分**: 約2,000-3,000行（型変換とラッパー）
- **純粋なC++実装**: 約15,000-20,000行（アルゴリズム実装）

---

## 8. 設計原則

### 8.1 分離の原則

1. **単方向依存**: MEX部分 → 純粋なC++実装（逆方向の依存は禁止）
2. **明確な境界**: `Inc/MEX/`が境界線
3. **独立性**: 純粋なC++実装はMATLABなしでコンパイル・実行可能
4. **再利用性**: 純粋なC++実装は他の環境（ROS、Arduinoなど）でも使用可能

### 8.2 実装コードの配置規則

| コードの種類 | 配置場所 | 例 |
|------------|---------|---|
| MATLABデータ型変換 | `Inc/MEX/` | `mex_type_conversion.hpp` |
| mexFunction実装 | `MEX/` | `mex_run_eskf.cpp` |
| アルゴリズム実装 | `Src/` | `eskf_core.cpp` |
| クラス定義 | `Inc/` (MEX/以外) | `eskf_core.hpp` |
| 独立ライブラリ | `Lib/` | `Matrix/matrix.hpp` |

---

## 9. 推奨事項

### 9.1 新しい実装を追加する場合

1. **純粋なC++実装を先に作成**: `Inc/`, `Src/`に配置
2. **MEXラッパーを後で作成**: `MEX/`, `Inc/MEX/`に配置
3. **依存関係を確認**: MEX部分が純粋なC++実装に依存することを確認

### 9.2 コードレビューのポイント

1. **MEX部分にアルゴリズム実装がないか確認**
2. **純粋なC++実装にMATLAB APIがないか確認**
3. **依存方向が正しいか確認**（MEX → 純粋なC++）

---

## 10. 課題と改善点

### 10.1 現状の課題

1. **MATLAB依存の混在**: `eskf_initializer.hpp/cpp`が無条件でMATLAB APIを使用
   - `initialize_eskf_from_matlab()`関数が`mxArray`を受け取る
   - **推奨対応**: この関数を`Inc/MEX/`に移動するか、条件付きコンパイルで対応
   - **現状**: MEX専用の初期化関数として機能しているため、実用上の問題はないが、設計上は分離が望ましい

2. **条件付きコンパイル**: `sensor_filter.hpp`が条件付きでMATLAB APIを使用
   - `#ifdef MATLAB_MEX_FILE`で条件付きに`mex.h`をインクルード
   - 非MEXビルド時は`printf`を使用するため問題なし
   - **評価**: これは許容可能な実装（条件付きコンパイルによる適切な分離）

3. **非推奨ファイル**: `mex_meukf_step.cpp`がまだ存在
   - 削除を検討（後方互換性確認後）

### 10.2 改善の方向性

1. **完全な分離**: 純粋なC++実装から全てのMATLAB依存を除去
2. **明確な境界**: `Inc/MEX/`を明確な境界として維持
3. **ドキュメント**: 依存関係を可視化した図を作成

---

## 11. まとめ

### 11.1 分離状況

✅ **良好な分離**:
- MEX部分と純粋なC++実装は明確に分離されている
- 依存関係は一方向（MEX → 純粋なC++）
- 純粋なC++実装の大部分はMATLAB非依存

⚠️ **改善が必要**:
- `eskf_initializer.hpp`と`sensor_filter.hpp`がMATLAB APIを使用している可能性
- 詳細な確認が必要

### 11.2 アーキテクチャの評価

現在のアーキテクチャは以下の利点を持っています：

1. **再利用性**: 純粋なC++実装は他の環境で使用可能
2. **テスト容易性**: 純粋なC++実装はMATLABなしでテスト可能
3. **保守性**: 明確な分離により保守が容易
4. **拡張性**: 新しいフィルター実装を追加しやすい

---

**関連ドキュメント**:
- `MEX/README.md`: MEXフォルダの役割と原則
- `README.md`: コードベース全体の構造
- `Lib/README.md`: 独立ライブラリの説明

