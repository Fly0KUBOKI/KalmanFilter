# 根本原因分析: Velocity RMSE 5.67 m/s の問題

## 発見された事実

### 1. 単体テストでは完全一致
- `debug_predict_postprocess.m`の実行結果:
  - Velocity difference: `0.000000e+00`
  - P difference: `0.000000e+00`
- **結論**: `mex_eskf_predict_postprocess`自体は正しく実装されている

### 2. バッチテストではVelocity RMSEが高い
- MATLAB実装: `0.5773 m/s`
- MEX実装: `5.67 m/s` (約10倍)

### 3. 処理フロー
```
ESKF.m predict() メソッド:
1. mex_adaptive_predict('predict', ...) を呼び出し
   → [p_new, v_new, q_new, ba_new, bg_new, P_new] を返す
   → 内部で float 精度で計算
   → vectorToMat() で double に変換して返す

2. [obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P] = deal(p_new, v_new, ...)
   → obj.v = v_new (double)

3. mex_eskf_predict_postprocess('postprocess', ...)
   → obj.v (double) を matToVector() で float に変換
   → 処理実行
   → vectorToMat() で double に変換して返す
```

## 根本原因の仮説

### 仮説1: `mex_adaptive_predict`の出力が既に問題を抱えている

#### 問題点
- `mex_adaptive_predict`は`float`精度で計算
- `ESKFCore::integrate_nominal()`が`float`精度で速度を更新
- 累積誤差が大きくなる可能性

#### 検証方法
- `mex_adaptive_predict`の出力（`v_new`）をMATLAB実装と比較
- 実際のシミュレーションデータを使用して検証

### 仮説2: 数値精度の累積誤差

#### 問題点
- `mex_adaptive_predict`: `double` → `float` → `double`
- `mex_eskf_predict_postprocess`: `double` → `float` → `double`
- 各変換で精度損失が発生

#### 影響
- 単体テストでは1回だけの実行なので問題なし
- バッチテストでは何千回も実行されるため、累積誤差が大きくなる

### 仮説3: `mex_adaptive_predict`とMATLAB実装の差分

#### 可能性
- `ESKFCore::integrate_nominal()`の実装がMATLAB実装と異なる
- 特に速度の更新ロジックに問題がある可能性

## 推奨される検証手順

### ステップ1: `mex_adaptive_predict`の出力を検証
```matlab
% debug_mex_adaptive_predict.m
% mex_adaptive_predictの出力をMATLAB実装と比較
```

### ステップ2: 実際のシミュレーションデータを使用
```matlab
% debug_predict_with_real_data.m
% 実際のシミュレーションデータを使用して検証
```

### ステップ3: 数値精度の問題を解決
- 可能であれば、`mex_adaptive_predict`内で`double`を使用
- または、MATLAB関数を直接呼び出して計算を実行

## 次のステップ

1. `debug_predict_with_real_data.m`を実行して、実際のシミュレーションデータで検証
2. `mex_adaptive_predict`の出力（特に`v_new`）をMATLAB実装と比較
3. 数値精度の問題を解決

