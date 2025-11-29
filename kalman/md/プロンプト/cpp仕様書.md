## やること
cppのプログラムをリファクタリングし、それにmatlabプログラムも対応させたい

## 詳細
cppのプログラムは状態量に依存しない計算プログラムとして機能させたい
そのため、accelやmagといった状態に依存させるのではなく、
あくまで共分散更新や、クォータ二オンの計算など、計算をする関数として用いたい
各状態量に応じた処理はmatlabプログラム側で行う
math.hの関数のような使い方ができるようにしたい
計算機としての関数は
引数の一つ目が計算機に代入するデータをまとめた行列、引数の二つ目が計算機から出力される行列としたい
必要に応じて、行列や構造体といった形を取る
なるべく引数はinput outputのみにしたい

## やること
kalman\md\CPP_MATLAB_DEPENDENCY_ANALYSIS.mdを読む
cppのプログラムをリファクタリングし、それにmatlabプログラムも対応させたい

## 詳細
cppのプログラムは状態量に依存しない計算プログラムとして機能させたい
そのため、accelやmagといった状態に依存させるのではなく、
あくまで共分散更新や、クォータ二オンの計算など、計算をする関数として用いたい
各状態量に応じた処理はmatlabプログラム側で行う
計算機としての関数は
引数の一つ目が計算機に代入するデータをまとめた行列、引数の二つ目が計算機から出力される行列としたい
必要に応じて、行列や構造体といった形を取る
なるべく引数はinput outputのみにしたい

## 注意事項
floatを基本的に使用する
100以下の整数はuint8_tを使用

## やること
より、安定して滑らかな高速に処理できるフィルタを作成するための改善計画を作成する
推定の改善案も参考にして

## 推定の改善案
ZUPT (静止更新) 追加	誤差低減	update_zero_vel 関数を追加。静止判定ロジックを実装。

姿勢更新のUKF化 (MEUKF)	安定性	update_accel, update_mag をEKFからUnscented Transformベースに変更。誤差状態空間(3D)でシグマ点を生成する。

適応型 Q (Adaptive Q)	滑らかさ	IMUの信号強度に基づき、予測ステップの Q を動的にスケーリングする処理を追加。
SR形式 / 全体MEX化	高速化	共分散 P ではなく S を持ち回るように変更。可能ならループ全体をC++へ移植。

Plan: ESKF Improvement (ZUPT, MEUKF, Adaptive Q, Full MEX)
This plan outlines the steps to enhance the ESKF with Zero Velocity Update (ZUPT) for drift reduction, Manifold UKF (MEUKF) for stable attitude estimation, Adaptive Q for smoothness, and a Full MEX implementation for high-speed processing.

Steps
Implement ZUPT (Zero Velocity Update)

Add check_stationary method to ESKF.m (using accel variance/gyro magnitude).
Add update_zupt to ESKF.m that calls a velocity update with zero observation.
Integrate into run_simulation.m loop.
Implement Adaptive Q

Modify ESKF.m's predict method to accept an optional scaling factor.
Calculate scaling factor 
α
α based on IMU stability (e.g., 
∣
∣
a
∣
∣
≈
g
∣∣a∣∣≈g) in run_simulation.m.
Dynamically scale process noise covariance 
Q
t
=
α
Q
n
o
m
i
n
a
l
Q 
t
​
 =αQ 
nominal
​
 .
Develop MEUKF (Manifold Error UKF) for Attitude

Create mex_meukf_update.cpp to handle quaternion manifold operations.
Implement sigma point generation in error space (3D rotation vector) and map to quaternion via exponential map.
Replace update_accel/update_mag in ESKF.m to use this new MEX function.
Full MEX Optimization (C++ Port)

Create a stateful C++ class ESKF_CPP mirroring the MATLAB logic.
Implement a "Batch MEX" driver (mex_run_eskf.cpp) that accepts all sensor arrays and runs the full loop in C++.
(Optional) Implement Square Root (SR) form using Cholesky decomposition within the C++ class for numerical stability.
Further Considerations
MEUKF Complexity: MEUKF requires careful handling of the "mean" on the manifold (Karcher mean). A simplified approach (resetting error mean to zero after update) is often sufficient for ESKF.
Full MEX Strategy: Instead of porting step-by-step, writing a single MEX function that takes all CSV data and returns all results is the fastest path to performance.
SR Form: Implementing Square Root form (updating 
S
S instead of 
P
P) is best done during the Full MEX rewrite to avoid duplicate work in MATLAB.
User Feedback**: Please confirm if you want to prioritize the Accuracy features (ZUPT, MEUKF) or the Speed features (Full MEX) first. I will proceed with the plan accordingly.******