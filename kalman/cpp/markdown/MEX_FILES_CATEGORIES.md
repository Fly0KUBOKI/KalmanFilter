# MEXファイル カテゴリ別分類

## 1. ESKF関連ファイル

### 1.1 初期化・コンストラクタ
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_eskf_constructor.cpp` | ESKF状態の初期化（静止データから姿勢推定） | なし | ✅ ビルド対象 |
| `mex_eskf_init.cpp` | 状態ハンドルの初期化 | なし | ✅ ビルド対象 |

### 1.2 状態管理
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_eskf_get_state.cpp` | 状態ハンドルから状態を取得 | なし | ✅ ビルド対象 |
| `mex_eskf_set_state.cpp` | 状態ハンドルに状態を設定 | なし | ✅ ビルド対象 |
| `mex_eskf_free.cpp` | 状態ハンドルの解放 | なし | ✅ ビルド対象 |

### 1.3 予測ステップ
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_adaptive_predict.cpp` | 予測ステップ（C++実装呼び出し） | `Src/ESKF/eskf_core.cpp` | ✅ ビルド対象 |
| `mex_eskf_predict_postprocess.cpp` | 予測後の後処理（速度ダンピング等） | なし | ❌ ソースなし（削除済み） |

### 1.4 更新ステップ
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_eskf_do_update.cpp` | センサー更新（do_cpp_updateのMEX化） | `mex_sensor_filter`, `mex_meukf_step_v2`, `mex_eskf_update_postprocess` | ✅ ビルド対象 |
| `mex_eskf_do_cpp_update.cpp` | do_cpp_updateの別実装 | `mex_sensor_filter`, `mex_meukf_step_v2`, `mex_eskf_update_postprocess` | ❓ ビルド対象外 |
| `mex_eskf_sensor_update.cpp` | 単一センサー更新 | `mex_sensor_preprocessor`, `mex_sensor_filter`, `mex_meukf_step_v2`, `mex_eskf_update_postprocess` | ✅ ビルド対象 |
| `mex_eskf_sensor_update_full.cpp` | センサー更新（完全版） | `mex_sensor_preprocessor` | ❓ ビルド対象外 |
| `mex_eskf_sensor_updates.cpp` | 複数センサー更新 | 不明 | ✅ ビルド対象 |
| `mex_eskf_sensor_updates_full.cpp` | 複数センサー更新（完全版） | `mex_sensor_preprocessor`, `mex_eskf_do_update` | ✅ ビルド対象 |
| `mex_eskf_update_postprocess.cpp` | 更新後の後処理 | なし | ❌ ソースなし（削除済み） |

### 1.5 ステップ実行
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_eskf_step.cpp` | 統合ESKFステップ（予測+更新） | `Inc/MEUKF/unified_filter.hpp`, `Src/MEUKF/unified_filter.cpp`, `Src/MEUKF/meukf_core.cpp` | ✅ ビルド対象 |
| `mex_eskf_step_handle.cpp` | 状態ハンドル版ステップ | `mex_unified_filter` | ✅ ビルド対象 |
| `mex_run_eskf.cpp` | ESKF.m完全置き換え | `mex_adaptive_predict`, `mex_eskf_predict_postprocess`, `mex_eskf_sensor_updates_full`, `mex_eskf_zupt`, `mex_filter_management`, `mex_eskf_constructor` | ✅ ビルド対象 |
| `mex_eskf_full.cpp` | 完全MEX化ESKF | `mex_adaptive_predict`, `mex_eskf_predict_postprocess` | ✅ ビルド対象 |

### 1.6 ユーティリティ
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_eskf_math.cpp` | ESKF数学関数群 | `Inc/ESKF/eskf_math.hpp`, `Src/ESKF/eskf_math.cpp` | ✅ ビルド対象 |
| `mex_eskf_helper.cpp` | ESKFヘルパー関数 | `Inc/ESKF/eskf_helper.hpp` | ❓ ビルド対象外 |
| `mex_eskf_core.cpp` | ESKFコア関数（legacy） | 不明 | ❌ スキップ（locked） |
| `mex_eskf_core_v2.cpp` | ESKFコア関数（v2） | 不明 | ❓ ビルド対象外 |
| `mex_eskf_zupt.cpp` | Zero Velocity Update | なし | ❌ ソースなし（削除済み） |
| `mex_eskf_reset.mexw64` | リセット機能 | 不明 | ❌ ソースなし |

## 2. フィルタコア

### 2.1 基本フィルタ
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_kalman_filter_core.cpp` | 基本カルマンフィルタ | なし | ✅ ビルド対象 |
| `mex_kf_core.cpp` | KFコア | 不明 | ❓ ビルド対象外 |
| `mex_ekf.cpp` | 拡張カルマンフィルタ | `Src/EKF/ekf_linear_update.cpp` | ✅ ビルド対象 |

### 2.2 UKF
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_ukf.cpp` | Unscented Kalman Filter | `Src/EKF/ekf_linear_update.cpp` | ✅ ビルド対象 |
| `mex_ukf_sigma_points.cpp` | UKFシグマポイント生成 | `Src/UKF/ukf_sigma_points.cpp` | ✅ ビルド対象 |
| `mex_ukf_update.cpp` | UKF更新ステップ | なし | ✅ ビルド対象 |
| `mex_ukf_update_minimal.cpp` | UKF更新（最小版） | なし | ❓ ビルド対象外 |

### 2.3 MEUKF
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_meukf_step.cpp` | MEUKFステップ | `Src/MEUKF/meukf_core.cpp` | ✅ ビルド対象（`mex_meukf_step_v2`として） |
| `mex_unified_filter.cpp` | 統一フィルタインターフェース | `Src/MEUKF/unified_filter.cpp`, `Src/MEUKF/meukf_core.cpp` | ✅ ビルド対象 |

## 3. ユーティリティ

### 3.1 数学関数
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_eskf_math.cpp` | ESKF数学関数群 | `Inc/ESKF/eskf_math.hpp`, `Src/ESKF/eskf_math.cpp` | ✅ ビルド対象 |
| `mex_kalman_compute.cpp` | 統一計算関数群 | `Inc/Common/Math/quaternion_compute.hpp`, `Inc/Common/Math/rotation_compute.hpp` | ❓ ビルド対象外 |
| `mex_quaternion_lib.cpp` | クォータニオンライブラリ | `Inc/Common/Math/quaternion_lib.hpp` | ❌ スキップ（locked） |
| `mex_common_lib.cpp` | 共通ライブラリ | 不明 | ❓ ビルド対象外 |

### 3.2 センサー処理
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_sensor_filter.cpp` | センサーフィルタ（ノイズ推定等） | `Inc/Common/Sensor/sensor_filter.hpp` | ✅ ビルド対象 |
| `mex_sensor_preprocessor.cpp` | センサーデータ前処理 | なし | ✅ ビルド対象 |

### 3.3 フィルタ管理
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_filter_management.cpp` | フィルタ管理（発散検出、リセット） | `Inc/Common/Math/fixed_matrix.hpp` | ✅ ビルド対象 |
| `mex_filter_utils.cpp` | フィルタユーティリティ | 不明 | ❓ ビルド対象外 |

### 3.4 ヘルパー
| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_matlab_helpers.cpp` | MATLABヘルパー関数 | なし | ✅ ビルド対象 |
| `mex_type_conv.hpp` | 型変換ヘルパー（ヘッダー） | なし | インクルードのみ |

## 4. 完全実装

| ファイル | 役割 | 依存関係 | ビルド状況 |
|---------|------|---------|-----------|
| `mex_run_eskf.cpp` | ESKF.m完全置き換え | 多数のMEX関数 | ✅ ビルド対象 |
| `mex_eskf_full.cpp` | 完全MEX化ESKF | `mex_adaptive_predict`, `mex_eskf_predict_postprocess` | ✅ ビルド対象 |

## 凡例

- ✅ ビルド対象: `build_mex.m`でビルドされる
- ❌ スキップ: `build_mex.m`で明示的にスキップ
- ❓ ビルド対象外: `build_mex.m`に記載なし
- ❌ ソースなし: バイナリは存在するがソースコードがない



