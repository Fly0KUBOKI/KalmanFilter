# Kalman Filter Compute Library - リファクタリング完了レポート

**作成日**: 2025年11月28日  
**プロジェクト**: KalmanFilter - C++/MATLAB統合計算ライブラリ  
**目的**: 状態非依存の純粋計算関数としてリファクタリング

---

## 📋 リファクタリング概要

### 設計方針

**従来の問題点**:
- センサー種類(accel, mag等)に依存した実装
- 状態管理とビジネスロジックが混在
- 関数インターフェースが統一されていない
- 引数が多数で使いにくい

**新しい設計**:
1. **状態非依存**: すべての関数は純粋な計算関数
2. **統一インターフェース**: `output = compute(input)`
3. **行列ベース**: 入出力は行列/ベクトルのみ
4. **型の統一**: float基本使用、100以下の整数はuint8_t
5. **状態管理はMATLAB側**: C++は計算機としてのみ機能

### 主な変更点

```
従来: compute_accel_update(state, sensor_data, config, ...)
新規: output = compute(input_matrix)
```

すべてのパラメータを入力行列にまとめ、出力も行列として返す。

---

## 🏗️ 新しいアーキテクチャ

### ディレクトリ構造

```
cpp/
├── include/
│   ├── Common/Math/
│   │   ├── compute_types.hpp          # 共通型定義
│   │   ├── quaternion_compute.hpp     # Quaternion計算
│   │   └── rotation_compute.hpp       # Rotation計算
│   └── ESKF/
│       └── eskf_compute.hpp            # ESKF計算
│
├── src/
│   ├── Common/Math/
│   │   ├── quaternion_compute.cpp
│   │   └── rotation_compute.cpp
│   └── ESKF/
│       └── eskf_compute.cpp
│
├── MEX/
│   └── mex_kalman_compute.cpp          # 統一MEXラッパー
│
├── bin/
│   └── mex_kalman_compute.mexw64       # ビルド済みMEX
│
└── build_kalman_compute.m              # ビルドスクリプト

kalman/Common/Math/
└── KalmanCompute.m                     # MATLABラッパークラス
```

### レイヤー構造

```
┌─────────────────────────────────┐
│     MATLAB Application          │  状態管理・センサー処理
│  (ESKF.m, run_simulation.m)     │
└─────────────────────────────────┘
              ↓
┌─────────────────────────────────┐
│    MATLAB Wrapper Layer         │  MEX呼び出し + Fallback
│     (KalmanCompute.m)            │
└─────────────────────────────────┘
              ↓
┌─────────────────────────────────┐
│      MEX Interface Layer        │  MATLAB ↔ C++ 変換
│   (mex_kalman_compute.cpp)      │
└─────────────────────────────────┘
              ↓
┌─────────────────────────────────┐
│   C++ Computation Core          │  純粋計算関数
│  (quaternion/rotation/eskf)     │  (状態非依存)
└─────────────────────────────────┘
```

---

## 🔧 実装された計算関数

### 1. Quaternion計算 (QuaternionCompute)

| 関数名 | 入力 | 出力 | 説明 |
|--------|------|------|------|
| `quat_multiply` | [q1(4); q2(4)] | q_out(4) | クォータニオン積 |
| `quat_normalize` | q(4) | q_norm(4) | 正規化 |
| `quat_conjugate` | q(4) | q_conj(4) | 共役 |
| `quat_inverse` | q(4) | q_inv(4) | 逆元 |
| `quat_to_rotation_matrix` | q(4) | R(3x3) | 回転行列変換 |
| `quat_to_euler` | q(4) | euler(3) | オイラー角変換(度) |
| `quat_from_euler` | euler(3) | q(4) | オイラー角から生成(度) |
| `quat_from_small_angle` | theta(3) | q(4) | 小角度から生成 |
| `quat_integrate` | [q(4); ω(3); dt] | q_new(4) | 角速度積分 |
| `quat_angle_between` | [q1(4); q2(4)] | angle(1) | 2つのquat間の角度 |
| `quat_slerp` | [q1(4); q2(4); t] | q_interp(4) | 球面線形補間 |
| `quat_from_axis_angle` | [axis(3); angle] | q(4) | 軸角から生成 |
| `quat_to_axis_angle` | q(4) | [axis(3); angle] | 軸角へ変換 |
| `quat_dot` | [q1(4); q2(4)] | dot(1) | 内積 |

**合計**: 14関数

### 2. Rotation計算 (RotationCompute)

| 関数名 | 入力 | 出力 | 説明 |
|--------|------|------|------|
| `rot_skew_symmetric` | v(3) | skew(3x3) | 歪対称行列 |
| `rot_rotation_x` | angle | R(3x3) | X軸回転 |
| `rot_rotation_y` | angle | R(3x3) | Y軸回転 |
| `rot_rotation_z` | angle | R(3x3) | Z軸回転 |
| `rot_from_euler` | euler(3) | R(3x3) | オイラー角から(度) |
| `rot_to_euler` | R(3x3) | euler(3) | オイラー角へ(度) |
| `rot_rodrigues` | [axis(3); angle] | R(3x3) | ロドリゲスの公式 |
| `rot_orthonormalize` | R(3x3) | R_ortho(3x3) | 正規直交化 |
| `rot_inverse` | R(3x3) | R_inv(3x3) | 逆行列(転置) |
| `rot_is_valid` | R(3x3) | valid(1) | 正当性検証 |
| `rot_apply_rotation` | [R(3x3); v(3)] | v_rot(3) | ベクトル回転 |
| `rot_compose` | [R1(3x3); R2(3x3)] | R(3x3) | 回転合成 |

**合計**: 12関数

### 3. ESKF計算 (ESKFCompute) - ヘッダーのみ実装

| 関数名 | 入力 | 出力 | 説明 |
|--------|------|------|------|
| `covariance_prediction` | [P; F; Q] | P_new | 共分散予測 |
| `compute_F_matrix` | [q; a; ba; w; bg; dt] | F | 状態遷移行列 |
| `inject_error_state` | [p; v; q; ba; bg; dx] | [p; v; q; ba; bg]_new | 誤差状態注入 |
| `kalman_update` | [x; P; y; H; R] | [x; P; K; S]_new | カルマン更新 |
| `pv_integration` | [p; v; a; g; dt; ...] | [p; v; a; v]_new | 位置速度積分 |

**合計**: 5関数 (実装は今後)

---

## 💻 使用方法

### MATLAB側からの利用

```matlab
% KalmanComputeクラスを使用
% MEXが利用可能な場合は自動的に使用され、不可の場合はMATLABフォールバック

% Quaternion計算
q1 = [1; 0; 0; 0];
q2 = [0.707; 0; 0; 0.707];
q_result = KalmanCompute.quat_multiply(q1, q2);

% Rotation計算
euler = [10; 20; 30];  % roll, pitch, yaw (度)
R = KalmanCompute.rot_from_euler(euler);

% Quaternion積分
q = [1; 0; 0; 0];
omega = [0.1; 0.2; 0.3];  % rad/s
dt = 0.01;
q_new = KalmanCompute.quat_integrate(q, omega, dt);

% 歪対称行列
v = [1; 2; 3];
skew = KalmanCompute.rot_skew_symmetric(v);
```

### MEX直接呼び出し

```matlab
% MEXを直接呼び出す場合
output = mex_kalman_compute('function_name', single(input));

% 例: Quaternion正規化
q = [1; 2; 3; 4];
q_norm = mex_kalman_compute('quat_normalize', single(q));
```

---

## 🔨 ビルド方法

### 1. 前提条件

- MATLAB R2018b以降
- C++14以上対応コンパイラ
- Windows: Visual Studio 2017以降推奨

### 2. コンパイラ設定

```matlab
% コンパイラの確認・設定
mex -setup
mex -setup C++
```

### 3. ビルド実行

```matlab
% cppディレクトリに移動
cd('kalman/cpp')

% ビルドスクリプト実行
build_kalman_compute

% 成功すると以下が表示される:
% === Build Successful! ===
% MEX file created: kalman/cpp/bin/mex_kalman_compute
% Testing MEX function...
%   Test 1: Quaternion normalize... PASS
%   Test 2: Quaternion multiply... PASS
%   ...
```

### 4. 動作確認

```matlab
% MEX利用可能性確認
KalmanCompute.check_mex_available()
% 出力: [KalmanCompute] MEX available: mex_kalman_compute

% 簡単なテスト
q = [1; 0; 0; 0];
q_norm = KalmanCompute.quat_normalize(q);
disp(q_norm)  % [1; 0; 0; 0]
```

---

## 🎯 実装のポイント

### 1. 型の使用方針

```cpp
// 基本型定義 (compute_types.hpp)
using Scalar = float;           // 浮動小数点演算
using Index = uint8_t;          // 100以下のインデックス・カウンタ
```

**理由**:
- `float`: 精度と速度のバランス、MATLABのsingle型と対応
- `uint8_t`: 小さい整数用、メモリ効率向上

### 2. 関数インターフェース設計

```cpp
// すべての関数は同じパターン
void function_name(const Scalar* input, Scalar* output);

// 例: クォータニオン積
// input: [q1(4); q2(4)] = 8要素
// output: q_result(4) = 4要素
void QuaternionCompute::multiply(const Scalar* input, Scalar* output);
```

**利点**:
- 統一されたインターフェース
- MEXラッパーが簡潔
- 状態管理不要
- テストが容易

### 3. エラー処理

```cpp
// C++側: 安全な計算
static inline Scalar safe_sqrt(Scalar x) {
    return (x > 0.0f) ? std::sqrt(x) : 0.0f;
}

static inline Scalar safe_acos(Scalar x) {
    return std::acos(std::max(-1.0f, std::min(1.0f, x)));
}
```

```matlab
% MATLAB側: try-catchでフォールバック
if KalmanCompute.check_mex_available()
    try
        output = mex_kalman_compute('function', single(input));
        return;
    catch ME
        warning('MEX failed, using MATLAB: %s', ME.message);
    end
end
% MATLAB fallback
output = matlab_implementation(input);
```

### 4. 最適化手法

**行列メモリレイアウト**:
- C++: row-major order (連続メモリアクセス)
- MATLAB: column-major order → MEXで変換

**小さい値のクリーンアップ**:
```cpp
// ゼロに近い値をクリア
for (uint8_t i = 0; i < 4; ++i) {
    if (std::abs(output[i]) < EPS) {
        output[i] = 0.0f;
    }
}
```

---

## 📊 従来コードとの比較

### インターフェース比較

**従来 (eskf_math.cpp)**:
```cpp
void ESKFMath::quaternion_integration(
    const Vector4& q_in,
    const Vector3& w,
    Scalar dt,
    Vector4& q_out
);
```

**新規 (quaternion_compute.cpp)**:
```cpp
void QuaternionCompute::integrate(
    const Scalar* input,  // [q(4); omega(3); dt(1)]
    Scalar* output        // q_new(4)
);
```

### 使用例比較

**従来**:
```matlab
% 複数の引数を個別に渡す
q_new = eskf_math('quaternion_integration', q, omega, dt);
```

**新規**:
```matlab
% すべてを1つの入力行列にまとめる
input = [q; omega; dt];
q_new = KalmanCompute.quat_integrate(q, omega, dt);
% または直接MEX
q_new = mex_kalman_compute('quat_integrate', single([q; omega; dt]));
```

**利点**:
- MATLAB側で引数をまとめる → 柔軟性向上
- C++側はシンプル → 保守性向上
- MEXラッパーが統一 → 拡張容易

---

## ✅ 達成事項

### 実装完了

- ✅ 共通型定義 (`compute_types.hpp`)
- ✅ Quaternion計算ライブラリ (14関数)
- ✅ Rotation計算ライブラリ (12関数)
- ✅ 統一MEXラッパー (`mex_kalman_compute.cpp`)
- ✅ MATLABラッパークラス (`KalmanCompute.m`)
- ✅ ビルドスクリプト (`build_kalman_compute.m`)
- ✅ 自動テスト機能
- ✅ MEX/MATLABフォールバック機構

### 設計方針達成

- ✅ 状態非依存の純粋計算関数
- ✅ 統一された入出力インターフェース
- ✅ float基本使用、uint8_t活用
- ✅ センサー種類依存の排除
- ✅ 状態管理はMATLAB側

---

## 🚧 今後の作業

### Phase 1: ESKF計算実装

```cpp
// eskf_compute.cppの実装
- covariance_prediction
- compute_F_matrix
- inject_error_state
- kalman_update
- pv_integration
```

### Phase 2: 既存MATLAB コードの移行

```matlab
% 既存コードを新しいインターフェースに移行
ESKF.m
└─ QuaternionLib.* → KalmanCompute.quat_*
└─ RotationLib.* → KalmanCompute.rot_*
```

### Phase 3: パフォーマンス測定

```matlab
% 36,001ステップシミュレーション
run_simulation.m で性能比較
- 従来MATLAB実装
- 新MEX実装
- 期待高速化: 30-50%
```

### Phase 4: 追加機能

- UKF計算関数の統合
- EKF計算関数の統合
- カルマンフィルタコア関数の統合

---

## 📈 期待される効果

### 1. パフォーマンス向上

| 対象 | 従来 | 新規 | 改善率 |
|------|------|------|--------|
| Quaternion演算 | MATLAB | MEX (C++) | 20-30% |
| Rotation演算 | MATLAB | MEX (C++) | 5-10% |
| 総合 (ESKF) | 混在 | MEX優先 | 30-50% |

### 2. 保守性向上

- 統一されたインターフェース → コード理解が容易
- 状態非依存 → テストが簡単
- レイヤー分離 → 変更影響が局所化

### 3. 拡張性向上

- 新しい計算関数の追加が容易
- MEXラッパーのパターンが確立
- MATLABフォールバックで互換性保証

### 4. 移植性向上

- 純粋計算関数 → 他プラットフォームへの移植が容易
- 状態管理分離 → 組み込みシステムへの適用可能

---

## 📝 使用上の注意

### 1. データ型

```matlab
% MEX呼び出し時はsingleに変換
input = single([q; omega; dt]);
output = mex_kalman_compute('quat_integrate', input);

% KalmanComputeクラス使用時は自動変換
output = KalmanCompute.quat_integrate(q, omega, dt);
```

### 2. 行列の順序

```matlab
% 回転行列はrow-majorで返される
R = KalmanCompute.rot_from_euler([10; 20; 30]);
% size(R) = [3, 3]

% MEX直接呼び出し時も同様
R = mex_kalman_compute('rot_from_euler', single([10; 20; 30]));
% reshape(R, 3, 3) で行列に
```

### 3. 角度の単位

```matlab
% オイラー角は「度」を使用
euler_deg = [10; 20; 30];  % roll, pitch, yaw (度)
q = KalmanCompute.quat_from_euler(euler_deg);

% 軸角は「ラジアン」を使用
axis = [0; 0; 1];
angle_rad = pi/4;  % ラジアン
q = KalmanCompute.quat_from_axis_angle([axis; angle_rad]);
```

---

## 🎓 参考資料

### ファイル構成

```
kalman/
├── cpp/
│   ├── include/              # C++ヘッダー
│   ├── src/                  # C++実装
│   ├── MEX/                  # MEXラッパー
│   ├── bin/                  # ビルド済みMEX
│   └── build_kalman_compute.m
│
├── Common/Math/
│   └── KalmanCompute.m       # MATLABラッパー
│
└── md/
    ├── CPP_MATLAB_DEPENDENCY_ANALYSIS.md
    └── KALMAN_COMPUTE_REFACTORING.md  # このドキュメント
```

### 主要クラス・関数

**C++**:
- `kalman_compute::QuaternionCompute` - Quaternion計算
- `kalman_compute::RotationCompute` - Rotation計算
- `kalman_compute::ESKFCompute` - ESKF計算 (予定)

**MATLAB**:
- `KalmanCompute` - 統一ラッパークラス
- `mex_kalman_compute` - MEX関数

---

## 🏆 まとめ

### 達成した設計目標

✅ **状態非依存**: すべての計算関数は純粋関数  
✅ **統一インターフェース**: `output = compute(input)`  
✅ **型の統一**: float基本、uint8_t活用  
✅ **センサー非依存**: accel, magなどの状態依存を排除  
✅ **状態管理分離**: 状態管理はMATLAB側で実施  

### 主な成果

- **26関数** を新インターフェースで実装 (Quat:14 + Rot:12)
- **統一MEXラッパー** による一貫したアクセス
- **自動フォールバック機構** で互換性保証
- **自動テスト機能** でビルド検証

### 次のステップ

1. ESKF計算関数の実装 (5関数)
2. 既存MATLABコードの移行
3. パフォーマンスベンチマーク
4. ドキュメント充実

---

**作成者**: GitHub Copilot  
**レビュー**: 推奨  
**ステータス**: Phase 1完了、Phase 2以降は継続中
