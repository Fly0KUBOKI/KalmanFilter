#pragma once

#include <cstdint>

namespace kalman_compute {

// 基本型定義
using Scalar = float;           // 浮動小数点演算はfloatを使用
using Index = uint8_t;          // 100以下のインデックス・カウンタ

// 計算関数の共通インターフェース規約:
// - すべての関数は状態を持たない純粋な計算関数
// - 第1引数: 入力データ行列/ベクトル
// - 第2引数: 出力データ行列/ベクトル
// - センサー種類などの状態依存処理はMATLAB側で実施
// - 計算関数は数学演算のみを実行

} // namespace kalman_compute
