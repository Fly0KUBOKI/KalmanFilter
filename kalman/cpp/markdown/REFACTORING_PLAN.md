# 超大規模リファクタリング計画

## 目標

1. **Lib構造の確立**: 独立したライブラリとして完結
2. **Eigenの廃止**: 全てfixed_matrix.hppベースに統一
3. **型の統一**: float型、uint8_t型の徹底
4. **インターフェースの統一**: init/update/getEuler/reset API
5. **動的メモリの廃止**: 全て静的配列
6. **モダンC++の廃止**: C++03互換

## ⚠️ 重要な設計原則

### MEXフォルダの役割

**MEXフォルダには実装コードを記述しない**

- `MEX/`フォルダは**ラッパーコードのみ**を含む
- 実装ロジック（アルゴリズム）は全て`Inc/`, `Src/`, `Lib/`に配置
- MEXファイルの役割：
  - MATLABデータ型とC++データ型の変換（ヘルパー関数）
  - `mexFunction`エントリーポイント
  - `Inc/`, `Src/`, `Lib/`の関数呼び出し

- **許可されるコード**：
  - MATLAB配列 ↔ C++型の変換関数（`get_vec3`, `set_vec3`など）
  - `mexFunction`の実装
  - エラーチェックとバリデーション

- **禁止されるコード**：
  - アルゴリズム実装（カルマンフィルタ計算、行列演算など）
  - ビジネスロジック
  - 状態管理

### 独立性の確保

**`Inc/`, `Src/`, `Lib/`は他のコンパイル環境でも使用可能**

- `Inc/`, `Src/`, `Lib/`をコンパイルすれば、そのまま他の環境（Arduino、ROS、スタンドアロンC++など）で使用可能
- MEXフォルダに依存しない設計
- インクルードパス: `-I Inc/`
- ソースファイル: `Src/**/*.cpp`
- ライブラリ: `Lib/**/*.hpp`

## フェーズ構成

```
Phase 1: Lib基盤構築 (低リスク)
    ↓
Phase 2: 型・メモリ修正 (中リスク)
    ↓
Phase 3: Eigen廃止 (高リスク)
    ↓
Phase 4: インターフェース統一 (高リスク)
    ↓
Phase 5: 統合・最適化 (最終)
```

---

## Phase 1: Lib基盤構築

### 1.1 新ディレクトリ構造

```
cpp/
├── Lib/
│   ├── Matrix/           # 静的行列ライブラリ
│   │   ├── matrix.hpp    # メインヘッダ
│   │   ├── matrix_ops.hpp # 行列演算
│   │   └── cholesky.hpp  # コレスキー分解
│   ├── Quaternion/       # クォータニオンライブラリ
│   │   └── quaternion.hpp
│   ├── KalmanCore/       # カルマンフィルタ基盤
│   │   ├── kf_gain.hpp   # ゲイン計算
│   │   └── kf_update.hpp # 更新関数
│   ├── EKF/              # 拡張カルマンフィルタ
│   ├── UKF/              # アンセンテッドカルマンフィルタ
│   └── ESKF/             # 誤差状態カルマンフィルタ
├── Inc/                  # インクルードファイル（実装含む）
│   ├── Common/
│   ├── EKF/
│   ├── ESKF/
│   ├── KF/
│   ├── MEUKF/
│   └── UKF/
├── Src/                  # ソースファイル（実装）
│   ├── Common/
│   ├── EKF/
│   ├── ESKF/
│   ├── MEUKF/
│   └── UKF/
├── MEX/                  # MATLABインターフェース（ラッパーのみ）
│   └── [mex_*.cpp]       # 実装コードは含まない
└── build/
```

### 1.2 作業項目

| タスク | ファイル | 作業内容 |
|--------|---------|---------|
| 1.1.1 | `Lib/Matrix/matrix.hpp` | fixed_matrix.hppをコピー・改良 |
| 1.1.2 | `Lib/Matrix/matrix_ops.hpp` | math_utils.hppから行列演算抽出 |
| 1.1.3 | `Lib/Matrix/cholesky.hpp` | コレスキー分解を独立モジュール化 |
| 1.1.4 | `Lib/Quaternion/quaternion.hpp` | 3種類を統合・整理 |
| 1.1.5 | `Lib/KalmanCore/kf_gain.hpp` | カルマンゲイン計算 |

### 1.3 MEXラッパーの確認

**重要**: MEXファイルが実装コードを含んでいないことを確認

- ✅ `mex_ekf.cpp` → `Src/EKF/ekf_linear_update.cpp`を呼び出すのみ
- ✅ `mex_unified_filter.cpp` → `Src/MEUKF/unified_filter.cpp`を呼び出すのみ
- ✅ `mex_eskf_step.cpp` → `Src/MEUKF/unified_filter.cpp`を呼び出すのみ
- ❌ MEXファイル内にアルゴリズム実装があってはならない

### 1.4 検証

各ステップ後に:
```matlab
cd cpp/build
build_mex()
cd ../..
run_batch_10sets
```

---

## Phase 2: 型・メモリ修正

### 2.1 double→float変換

| ファイル | 変更箇所 | 優先度 |
|---------|---------|-------|
| `meukf_core.cpp` | `make_vector3(double x,y,z)` → `float` | 高 |
| `meukf_core.cpp` | `make_vector4()` → `float` | 高 |
| `meukf_core.cpp` | `vector3_norm()` 戻り値 → `float` | 高 |
| `unified_types.hpp` | `FilterInput::dt` → `float` | 高 |
| `unified_types.hpp` | 全ての`double` → `float` | 高 |

### 2.2 動的メモリ廃止

| ファイル | 変更箇所 | 対応 |
|---------|---------|------|
| `math_utils.hpp` | `median()` の `new/delete` | 固定配列 `float sorted[MAX_N]` |
| `sensor_filter.hpp` | 問題なし | - |

### 2.3 整数型の統一

200以下の整数にuint8_tを使用:

| ファイル | 変更箇所 |
|---------|---------|
| 全ファイル | ループカウンタ `int i` → `uint8_t i` (適切な場合) |
| 全ファイル | サイズ変数 → `uint8_t` |

**注意**: ループカウンタは符号付きが安全な場合もあるため慎重に

---

## Phase 3: Eigen廃止

### 3.0 MEXラッパーの確認（再確認）

**Phase 3開始前に必須**: すべてのMEXファイルがラッパーのみであることを確認

- MEXファイル内に実装コードが残っていないか確認
- すべての実装が`Inc/`, `Src/`, `Lib/`に移動されているか確認
- `Inc/`, `Src/`, `Lib/`のみでコンパイル可能か確認（MEXフォルダなしで）

### 3.1 対象ファイル

| ファイル | 依存内容 | 難易度 |
|---------|---------|-------|
| `EKF/ekf.hpp` | MatrixXd, VectorXd | 高 |
| `include/EKF/ekf.hpp` | 同上 | 高 |
| `include/UKF/ukf_sigma_points.hpp` | MatrixXf, VectorXf | 中 |
| `src/UKF/ukf_sigma_points.cpp` | LLT, SelfAdjointEigenSolver | 高 |
| `include/UKF/ukf_update.hpp` | std::function, Vector | 中 |
| `tests/compare_cholesky.cpp` | テスト用 (最後) | 低 |

### 3.2 EKF廃止手順

**現状**: EKF/ekf.hppはMEUKFで使用されていない

1. `include/EKF/ekf_core.hpp` (Eigen不使用版) が既に存在
2. EKF/ekf.hpp を削除またはinclude/EKF/ekf_core.hppに置換
3. ビルド・テスト

### 3.3 UKFシグマ点廃止手順

**現状**: MEUKFは独自のシグマ点生成を使用

1. `include/UKF/ukf_core.hpp` の `generate_sigma_points()` を確認 (Eigen不使用)
2. `ukf_sigma_points.hpp/cpp` の機能を `ukf_core.hpp` に統合
3. Eigen依存ファイルを削除
4. ビルド・テスト

### 3.4 コレスキー分解の統一

Eigen::LLTの代替:

```cpp
// Lib/Matrix/cholesky.hpp
template<uint8_t N, typename T = float>
bool cholesky(const Matrix<N,N,T>& A, Matrix<N,N,T>& L) {
    for (uint8_t i = 0; i < N; ++i) {
        for (uint8_t j = 0; j <= i; ++j) {
            T sum = T(0);
            for (uint8_t k = 0; k < j; ++k) {
                sum += L(i,k) * L(j,k);
            }
            if (i == j) {
                T diag = A(i,i) - sum;
                if (diag <= T(0)) return false;
                L(i,j) = std::sqrt(diag);
            } else {
                L(i,j) = (A(i,j) - sum) / L(j,j);
            }
        }
    }
    return true;
}
```

---

## Phase 4: インターフェース統一

### 4.1 目標API

```cpp
class Filter {
public:
    // 初期化
    uint8_t init();
    
    // メイン更新 (全センサー)
    // accel: [m/s^2], gyro: [deg/s], mag: [μT]
    uint8_t update(float accel[3], float gyro[3], float mag[3]);
    
    // 姿勢取得
    // angle: [roll, pitch, yaw] in [deg]
    // rate: [roll_rate, pitch_rate, yaw_rate] in [deg/s]
    uint8_t getEuler(float angle[3], float rate[3]);
    
    // リセット
    uint8_t reset();
};
```

### 4.2 オプション拡張API

```cpp
class FilterExtended : public Filter {
public:
    // GPS更新
    uint8_t updateGPS(float pos[3], float vel[3]);
    
    // 気圧更新
    uint8_t updateBaro(float altitude);
    
    // 位置取得
    uint8_t getPosition(float pos[3], float vel[3]);
    
    // 共分散取得
    uint8_t getCovariance(float* P, uint8_t size);
};
```

### 4.3 移行手順

1. `Filter` 基底クラスを作成
2. `MEUKFWrapper` で既存MEUKFをラップ
3. 既存MEXを新APIに対応
4. テスト
5. 他のフィルタ (EKF, ESKF, UKF) を同様にラップ

---

## Phase 5: 統合・最適化

### 5.1 不要ファイルの削除

| ファイル | 理由 |
|---------|------|
| `EKF/ekf.hpp` | Eigen依存、未使用 |
| `EKF/ekf.cpp` | 同上 |
| `include/UKF/ukf_sigma_points.hpp.eigen_working` | バックアップ |
| 旧パス `Common/Math/*.hpp` | include/に移行済み |

### 5.2 ドキュメント整備

1. `Lib/README.md` - ライブラリ使用方法
2. `Filter/MEUKF/README.md` - MEUKF使用方法
3. API仕様書

### 5.3 最終テスト

```matlab
% 10セットバッチテスト
run_batch_10sets

% 結果比較
% Roll RMSE < 0.3 deg
% Pitch RMSE < 0.3 deg
% Yaw RMSE < 1.0 deg
```

---

## 作業チェックリスト

### Phase 1
- [x] Lib/Matrix/matrix.hpp 作成
- [x] Lib/Matrix/cholesky.hpp 作成 (decomposition.hppとして)
- [x] Lib/Quaternion/quaternion.hpp 作成
- [x] Lib/KalmanCore/kf_gain.hpp 作成
- [ ] ビルド確認
- [ ] バッチテスト

### Phase 2
- [x] double→float変換 (meukf_core.cpp)
- [x] double→float変換 (unified_types.hpp)
- [x] 動的メモリ廃止 (math_utils.hpp)
- [ ] ビルド確認
- [ ] バッチテスト

### Phase 3
- [x] EKF/ekf.hpp 削除 (未使用確認済み、kalman_filters.hpp更新済み)
- [x] UKFシグマ点 Eigen廃止 (kalman_filters.hpp更新済み、ukf_core.hpp使用)
- [ ] コレスキー分解統一
- [ ] ビルド確認
- [ ] バッチテスト

### Phase 4
- [ ] Filterインターフェース作成
- [ ] MEUKFWrapper作成
- [ ] MEX更新
- [ ] ビルド確認
- [ ] バッチテスト

### Phase 5
- [ ] 不要ファイル削除
- [ ] ドキュメント整備
- [ ] 最終テスト

---

## リスク管理

### 高リスク作業

1. **Eigen廃止 (Phase 3)**: 数値精度への影響
   - 対策: 各ステップでバッチテスト、RMSE比較

2. **インターフェース変更 (Phase 4)**: 既存MEXへの影響
   - 対策: 段階的移行、旧APIの一時的維持

### ロールバック手順

各Phase完了時にgit tag:
```bash
git tag phase1-complete
git tag phase2-complete
# ...
```

問題発生時:
```bash
git checkout phase{n-1}-complete
```


