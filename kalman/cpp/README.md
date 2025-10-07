# KalmanFIlter_cpp

軽量な配列ベースのカルマンフィルタ実装（C++）。

このリポジトリは、STM32 や組み込み用途を想定した最小限のカルマンフィルタ実装を含んでいます。
ヘッダは固定最大サイズのバッファを使い、動的確保を避けることで小さな組み込み環境でも動作するように設計されています。

## 概要

主なファイル:
- `kalman_filter.hpp` - クラス定義、設定用マクロ、内部バッファの宣言
- `kalman_filter.cpp` - フィルタ本体の実装（Init / Update / EstimateNoise / GetData）

特徴:
- 配列のみで実装、動的メモリを使用しない（組み込み向け）
- オンラインのノイズ推定（EWMA）機能を持つ

## API（クラス: KalmanFilter）

公開メソッド:
- `void Init(uint8_t state_size, uint8_t obs_size);`
  - フィルタの実サイズを設定して内部バッファを初期化します

- `void Update();`
  - 現在の `prediction` / `system_matrix` / `observation_matrix` / `observation` を元に一ステップのフィルタ更新を行い、内部 `output` を更新します。

- `void EstimateNoise(float meas);`
  - 測定値を与えて観測ノイズ（R）とプロセスノイズ（Q）を EWMA 法でオンライン推定します。シンプルな自己適応が欲しい場合に呼び出します。

- `void GetData(float* out_states);`
  - フィルタ出力（状態推定）をユーザ提供の配列に書き出します。配列長は `state_size` と同じにしてください。

主要メンバ（ユーザが設定/読み出しするもの）:
- `float observation[OBS_SIZE_MAX]` - 観測値ベクトル（入力）
- `float prediction[STATE_SIZE_MAX]` - 予測値ベクトル（予測ステップの入力）
- `float system_matrix[...]` - システム遷移行列 F（row-major）
- `float observation_matrix[...]` - 観測行列 H（row-major）

マクロ（コンパイル時設定）:
- `NOISE_ALPHA` - ノイズ推定の EWMA α（既定 0.02）
- `KALMAN_USE_SMOOTHER` - スムーザーを有効化（true/false）
- `SMOOTHER_ALPHA` / `SMOOTHER_WINDOW_SIZE` - スムーザー調整パラメータ


## 使用例（単純な高度推定、状態 = [位置, 速度], 観測 = 位置）

下記は本リポジトリに含まれているクラスを利用する最小の使用例です（`main.cpp` の例）。

```cpp
#include "kalman_filter.hpp"
#include <iostream>

int main() {
    KalmanFilter kf;
    const uint8_t state_size = 2; // 位置, 速度
    const uint8_t obs_size = 1;   // 位置のみ観測
    kf.Init(state_size, obs_size);

    // 簡単な F (dt = 1)
    float dt = 1.0f;
    // F = [[1, dt],[0,1]] (row-major)
    kf.system_matrix[0 * state_size + 0] = 1.0f;
    kf.system_matrix[0 * state_size + 1] = dt;
    kf.system_matrix[1 * state_size + 0] = 0.0f;
    kf.system_matrix[1 * state_size + 1] = 1.0f;

    // H = [1, 0] （観測は位置のみ）
    kf.observation_matrix[0 * state_size + 0] = 1.0f;
    kf.observation_matrix[0 * state_size + 1] = 0.0f;

    // 初期予測（任意）
    kf.prediction[0] = 0.0f; // 位置
    kf.prediction[1] = 0.0f; // 速度

    // ループ例（受信した観測値 z を処理）
    float z = 10.0f; // 例: 受信した観測
    for (int t = 0; t < 10; ++t) {
        // 実運用ではここで kf.prediction をモデルに従って更新する（例えば運動方程式で予測）

        // 観測をセット
        kf.observation[0] = z + (float)t * 0.1f; // ダミー

        // オンラインノイズ推定（任意）
        kf.EstimateNoise(kf.observation[0]);

        // カルマン更新
        kf.Update();

        // 出力を取り出す
        float out[state_size];
        kf.GetData(out);
        std::cout << "t=" << t << " pos=" << out[0] << " vel=" << out[1] << "\n";
    }

    return 0;
}
```
