# C++ Directory Reorganization Report

**日付**: 2025年11月28日  
**作業内容**: cpp/ フォルダの機能別整理と構成の簡潔化

## 実行した作業

### 1. 新しいディレクトリ構造の作成

機能ごとに明確に分離した構成を実装：

```
cpp/
├── src/              # C++実装ソース (.cpp)
│   ├── KF/
│   ├── EKF/
│   ├── ESKF/
│   └── UKF/
│
├── include/          # C++ヘッダファイル (.hpp)
│   ├── Common/
│   │   ├── Math/
│   │   ├── Sensor/
│   │   └── Validation/
│   ├── KF/
│   ├── EKF/
│   ├── ESKF/
│   └── UKF/
│
├── mex/              # MEXラッパーソース (.cpp)
│
├── bin/              # ビルド成果物
│   └── prebuilt/     # プリビルド済MEXバイナリ (.mexw64)
│
├── build/            # ビルドスクリプトとログ
│   ├── build_mex.m
│   └── build_test.m
│
└── tests/            # テストスクリプト
```

### 2. ファイル移動の実施

| カテゴリ | 移動元 | 移動先 | ファイル数 |
|---------|-------|-------|-----------|
| ヘッダ (Common) | `Common/Math/*.hpp`<br>`Common/Sensor/*.hpp`<br>`Common/Validation/*.hpp` | `include/Common/Math/`<br>`include/Common/Sensor/`<br>`include/Common/Validation/` | 7個 |
| ヘッダ (KF) | `KF/*.hpp`<br>`KF/Core/*.hpp` | `include/KF/` | 2個 |
| ソース+ヘッダ (EKF) | `EKF/*.cpp`<br>`EKF/*.hpp` | `src/EKF/`<br>`include/EKF/` | 4個 |
| ソース+ヘッダ (ESKF) | `ESKF/*.cpp`<br>`ESKF/*.hpp` | `src/ESKF/`<br>`include/ESKF/` | 4個 |
| ソース+ヘッダ (UKF) | `UKF/Core/*.cpp`<br>`UKF/Core/*.hpp` | `src/UKF/`<br>`include/UKF/` | 6個 |
| MEXラッパー | `MEX/*.cpp` | `mex/` | 13個 |
| MEXバイナリ | `*.mexw64`<br>`MEX/*.mexw64` | `bin/prebuilt/` | 8個 |
| ビルドスクリプト | `build_mex.m`<br>`build_test.m` | `build/` | 2個 |

**合計移動ファイル数**: 46個

### 3. ビルドスクリプトの更新

`build/build_mex.m` を新しいディレクトリ構成に対応：

- **パス設定の変更**:
  - インクルードパス: `include/`, `include/Common/`, `include/KF/` 等
  - ソースパス: `src/ESKF/eskf_core.cpp` 等
  - 出力先: `bin/prebuilt/` に統一

- **簡潔化**:
  - 冗長なインクルードパス指定を削除
  - すべてのMEXビルドが同じ出力先を使用

### 4. サポートスクリプトの作成

#### `setup_cpp_path.m`
- MATLAB パスに `bin/prebuilt/` を追加
- 利用可能な MEX ファイルを自動検出・表示

#### `test_mex_loading.m`
- 8つの MEX ファイルの読み込みテスト
- パス設定の動作確認

### 5. ドキュメント更新

`README.md` を新構成に合わせて完全改訂：

- 新しいディレクトリ構成図を追加
- `setup_cpp_path()` と `test_mex_loading()` の使用方法を記載
- ビルド手順を `cd cpp/build; build_mex()` に更新
- トラブルシューティングセクションを簡潔化

### 6. クリーンアップ

空になった旧ディレクトリを削除：
- `Common/` (移動済)
- `EKF/` (移動済)
- `ESKF/` (移動済)
- `KF/` (移動済)
- `MEX/` (移動済)
- `UKF/` (移動済)

## 成果

### ✅ 達成項目

1. **明確な分離**: ソース、ヘッダ、MEXラッパー、バイナリが完全に分離
2. **見やすさ向上**: MEXバイナリが `bin/prebuilt/` に集約され、ルートディレクトリがすっきり
3. **ビルド管理の改善**: ビルドスクリプトとログが `build/` に集約
4. **保守性向上**: 機能ごと（KF, EKF, ESKF, UKF）にディレクトリが整理され、関連ファイルが近接
5. **拡張性**: `tests/` ディレクトリを予約し、将来のテスト追加に対応

### 📊 整理前後の比較

| 項目 | 整理前 | 整理後 |
|-----|-------|-------|
| トップレベルディレクトリ数 | 10個 | 5個 (bin, build, include, mex, src, tests) |
| MEXバイナリの配置 | ルートと`MEX/`に分散 | `bin/prebuilt/` に集約 |
| ビルドスクリプトの場所 | ルート | `build/` |
| ヘッダファイルの場所 | 各機能フォルダ内に分散 | `include/` に集約 |
| ソースファイルの場所 | 各機能フォルダ内 | `src/` に集約 |

### 🔧 MEX ファイルの状態

すべての MEX バイナリが `bin/prebuilt/` に正常に配置されています：

| MEX ファイル | サイズ | 最終更新 | 状態 |
|------------|-------|---------|------|
| mex_kf_interface.mexw64 | 839 KB | 2025/11/28 11:13 | ✓ OK |
| mex_ekf_interface.mexw64 | 839 KB | 2025/11/28 11:13 | ✓ OK |
| mex_eskf_core.mexw64 | 854 KB | 2025/11/28 11:12 | ✓ OK |
| mex_quaternion_lib.mexw64 | 27 KB | 2025/11/28 11:12 | ✓ OK |
| mex_ukf_sigma_points.mexw64 | 105 KB | 2025/11/28 11:12 | ✓ OK |
| mex_ukf_update.mexw64 | 985 KB | 2025/11/28 11:13 | ✓ OK |
| mex_kalman_filter_core.mexw64 | 829 KB | 2025/11/28 11:12 | ✓ OK |
| mex_common_lib.mexw64 | 28 KB | 2025/11/21 14:15 | ✓ OK |

**合計**: 8 個のMEXファイル、すべて利用可能

## 使用方法（更新後）

### MATLAB で MEX を使用する

```matlab
% 1. パス設定
cd cpp
setup_cpp_path()

% 2. MEX 読み込みテスト
test_mex_loading()

% 3. シミュレーション実行（自動でMEX使用）
cd ..
run_simulation()
```

### MEX を再ビルドする場合

```matlab
cd cpp/build
build_mex()
```

## 今後の推奨事項

1. **Git 管理**:
   - `bin/prebuilt/*.mexw64` を `.gitignore` に追加するか、Git LFS で管理
   - `build/` 内の一時ファイル（ログ等）も除外

2. **テストの拡張**:
   - `tests/` ディレクトリに単体テストを追加
   - CI/CD でのビルド・テスト自動化

3. **クロスプラットフォーム対応**:
   - CMake ベースのビルドシステム導入
   - macOS (.mexmaci64), Linux (.mexa64) 用の自動ビルド

4. **ドキュメント**:
   - `include/` 内のヘッダに Doxygen コメント追加
   - API リファレンスの自動生成

## 整理作業の影響範囲

### ✅ 影響なし（既存コードは変更不要）
- MATLAB 側のフィルタクラス (`KF.m`, `EKF.m`, `ESKF.m`, `UKF.m`)
- MEX 関数の呼び出しインターフェース（変更なし）
- 既存のシミュレーションスクリプト

### ⚠️ 要対応（一度だけ実行）
- MATLAB 起動時に `setup_cpp_path()` を実行してパス設定
- または `startup.m` に `addpath('cpp/bin/prebuilt')` を追加

## まとめ

cpp/ フォルダの整理により、以下を実現しました：

1. **視認性**: ソース、ヘッダ、バイナリが明確に分離
2. **保守性**: 機能ごとにファイルが整理され、関連コードが見つけやすい
3. **拡張性**: 新しい機能追加時のディレクトリ構成が明確
4. **一貫性**: ビルド出力先が `bin/prebuilt/` に統一

すべての MEX ファイルは正常に動作し、MATLAB から利用可能な状態です。
