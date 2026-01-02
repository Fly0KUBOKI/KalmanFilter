# KalmanFilter C++ リファクタリング: 要件と計画

## 📋 要件サマリー

### やること1: Markdown ドキュメント整理
- 全ての Markdown を読み取り、不要なものを削除
- 必要な情報を整理・統合

### やること2: C++ 大規模リファクタリング

#### 目標 API
```cpp
// スタンドアロン API（main から直接呼び出し可能）
uint8_t init()
uint8_t update(float accel[3], float gyro[3], float mag[3])  // [m/s², rad/s, μT]
uint8_t getEuler(float angle[3], float rate[3])              // [deg, deg/s]
uint8_t reset()
```

#### 要件
- ✅ MEX フォルダ：MATLAB 連携部分のみ
- ✅ Inc/Src/Lib をコンパイルすれば推定実行可能
- ✅ KF/EKF/UKF/ESKF/MEUKF を `Lib/*/inc/`, `Lib/*/src/` に標準化
- ✅ ファイル名を簡潔に（<15文字目標）
- ✅ API を統一設計
- ✅ インクルードパスを統合（マスターヘッダー）

#### 制約
- ❌ 可変配列・可変型は禁止（MEX 除く）→ 静的確保のみ
- ✅ 小数は `float`、小整数（≤200）は `uint8_t` 使用
- 📝 計画・進捗・結果を Markdown で記録

### Matrix & Quaternion
- ✅ ヘッダーライブラリのままで OK（`Lib/Matrix/`, `Lib/Quaternion/`）
- src ファイル不要

---

## 📅 詳細計画

**→ [REFACTORING_PLAN.md](REFACTORING_PLAN.md) を参照**

以下の 6 フェーズに分割：

| # | フェーズ | 内容 | 予定日 |
|---|---------|------|--------|
| 1 | Markdown 整理 | 過時ドキュメント削除（8ファイル）→ 5-6 に削減 | 2026-01-02 |
| 2 | ファイル構造標準化 | `Lib/*/inc/`, `Lib/*/src/` に統一、ファイル名簡潔化 | 2026-01-03 |
| 3 | API 統一化 | `Filter` 基底クラス、各フィルターが共通インターフェース | 2026-01-04 |
| 4 | インクルード統合 | `kalman_all.hpp` マスターヘッダー、CMakeLists.txt | 2026-01-04 |
| 5 | main() サンプル | CSV 読み込み→フィルター実行→出力 | 2026-01-05 |
| 6 | 型統一・最適化 | float/uint8_t 統一、動的メモリ禁止確認 | 2026-01-05 |

---

## 🎯 期待される成果

### ファイル構造
```
Lib/
  ├─ Matrix/fixed_matrix.hpp       (ヘッダーのみ)
  ├─ Quaternion/quat.hpp           (ヘッダーのみ)
  ├─ Common/{inc,src}              (フィルター共通)
  ├─ KF/{inc,src}                  (標準 KF)
  ├─ EKF/{inc,src}                 (拡張 KF)
  ├─ UKF/{inc,src}                 (Unscented KF)
  ├─ ESKF/{inc,src}                (Error State KF)
  ├─ MEUKF/{inc,src}               (Modified Error UKF)
  └─ README.md

Inc/
  └─ kalman_all.hpp                (マスターヘッダー)

examples/
  ├─ main_eskf.cpp                 (ESKF サンプル)
  └─ CMakeLists.txt

markdown/
  ├─ REFACTORING_PLAN.md           (本計画・進捗)
  ├─ CPP_INPUT_OUTPUT_SPEC.md      (MEX 仕様)
  └─ （その他 5-6 ファイル）
```

### ユーザーコード例
```cpp
#include "Inc/kalman_all.hpp"

int main() {
  kalman::ESKFFilter filter;
  
  // 初期化
  kalman::SensorData obs = {...};
  filter.init(obs, 5.0f);
  
  // メインループ
  for (int k = 0; k < n_samples; k++) {
    filter.update(obs);
    kalman::State state;
    filter.getState(state);
  }
  
  return 0;
}
```

---

**進捗状況**: フェーズ 0 計画完了 ✅
**次のステップ**: フェーズ 1（Markdown 整理）開始
