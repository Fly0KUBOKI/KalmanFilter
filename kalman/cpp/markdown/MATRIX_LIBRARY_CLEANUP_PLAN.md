# Matrixライブラリ整理計画

## 現状

`Lib/Matrix/`フォルダには以下の3つのファイルがあります：

1. ✅ **`fixed_matrix.hpp`** - **使用中**
   - `cmath_fx::Matrix<R, C, T>` - 全てのフィルタ実装で使用
   - `cmath_fx::FixedMatrix` - 動的サイズ行列

2. ⚠️ **`matrix.hpp`** - **未使用**
   - `lib::matrix::Mat<R, C, T>` - 未使用のライブラリ
   - `Lib/KalmanCore/gain.hpp`のみから参照されているが、`Lib/KalmanCore/gain.hpp`自体も未使用

3. ⚠️ **`decomposition.hpp`** - **未使用**
   - `lib::matrix`名前空間のコレスキー分解関数
   - `matrix.hpp`に依存
   - `Lib/KalmanCore/gain.hpp`のみから参照されているが、`Lib/KalmanCore/gain.hpp`自体も未使用

## コレスキー分解の使用状況

- **MEUKF**: `meukf_core.cpp`に独自の`cholesky3x3`と`cholesky3x3_robust`を実装
- **UKF**: `ukf_sigma_points.cpp`に独自の`chol_lower_raw`を実装
- **math_utils.hpp**: `FixedMatrix`用の`safe_cholesky`を実装

**結論**: `decomposition.hpp`の機能は実際には使用されていない（各フィルタが独自実装を使用）

## 提案

### オプション1: 未使用ファイルを削除（推奨）

`matrix.hpp`と`decomposition.hpp`を削除し、`fixed_matrix.hpp`のみを残す。

**メリット**:
- ファイル数が減り、管理が簡単になる
- 使用されていないコードを削除できる
- `Lib/Matrix/`フォルダがシンプルになる（`fixed_matrix.hpp`のみ）

**デメリット**:
- 将来的に`lib::matrix::Mat`を使いたい場合、再実装が必要

### オプション2: 保持

現状のまま保持する。

**メリット**:
- 将来の使用に備えられる

**デメリット**:
- 未使用コードが残る
- メンテナンスコストがかかる

## 推奨

**オプション1（削除）を推奨**します。理由：

1. `Lib/KalmanCore/gain.hpp`も未使用であり、`matrix.hpp`と`decomposition.hpp`を使用している唯一のファイル
2. 実際のフィルタ実装では`cmath_fx::Matrix`を使用しており、`lib::matrix::Mat`への移行予定はない
3. コレスキー分解は各フィルタが独自に実装しており、統一ライブラリの需要がない

## 実施手順（オプション1を選択した場合）

1. `Lib/Matrix/matrix.hpp`を削除
2. `Lib/Matrix/decomposition.hpp`を削除
3. `Lib/KalmanCore/gain.hpp`も削除（未使用のため）
4. `Lib/README.md`を更新
5. ドキュメントを更新

