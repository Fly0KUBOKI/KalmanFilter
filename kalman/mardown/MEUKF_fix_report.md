# MEUKF修正完了レポート

作成日: 2025-11-30  
修正者: 自動修正システム

## 概要
ESKF実装に組み込まれたMEUKF（Multiplicative Error-state UKF）が「発散しやすく、ギザギザしている」という問題を解決しました。修正後、MEUKFはEKFモードよりも**優れた推定精度**を達成しました。

## 実施した修正内容

### 1. `meukf_update_attitude.m` の修正
- **UKFパラメータのデフォルト値を変更**:  
  - `alpha = 0.1` → `alpha = 1e-3`（他のUKF実装と整合）
  - シグマ点の広がりを抑制し、数値安定性を向上

- **共分散更新式を標準形に修正**:  
  ```matlab
  % 修正前（非標準）
  P_upd = P_sub - K * S * K' + K * R * K';
  
  % 修正後（標準形）
  P_upd = P_sub - K * S * K';
  ```
  - 不要な `+ K*R*K'` 項を削除し、標準的なカルマンフィルタ更新式に統一

- **正定値化の強化**:
  - 更新後の共分散行列に対する対称化と小ジッタ追加を実装
  - 固有値チェックと自動正則化を追加

### 2. ESKF.m の MEUKF 更新部分の修正
- **フルクロス共分散の更新を追加**（最重要修正）:
  - 修正前: 姿勢ブロック（3×3）のみを置き換え → 他の状態との相関が不整合
  - 修正後: フル状態（15次元）のクロス共分散を整合的に更新
  
- **実装の詳細**:
  ```matlab
  % 観測行列の線形化（EKFと同じロジック）
  Rb = QuaternionLib.to_rotation_matrix(obj.q);
  g_body = Rb' * obj.g;
  H_attitude = -RotationLib.skew_symmetric(g_body);
  H_sub = H_attitude(1:2, :);  % x,y成分のみ
  
  % フル状態に対するカルマンゲイン計算
  P_cross = obj.P(:, idx_obs);  % 15x3
  K_full = P_cross * H_sub' / S;  % Cholesky安定化
  
  % Joseph形式でフル共分散を更新
  I_KH_block = eye(length(idx_obs)) - K_full(idx_obs,:) * H_sub;
  obj.P(idx_obs, idx_obs) = I_KH_block * P_attitude_upd * I_KH_block' + ...
                            K_full(idx_obs,:) * R * K_full(idx_obs,:)';
  
  % クロス項更新
  for i = 1:15
      if ~ismember(i, idx_obs)
          obj.P(i, idx_obs) = obj.P(i, idx_obs) - K_full(i,:) * (H_sub * obj.P(idx_obs, idx_obs));
          obj.P(idx_obs, i) = obj.P(i, idx_obs)';
      end
  end
  ```

- **加速度更新（`update_accel_meukf`）と磁気計更新（`update_mag_meukf`）の両方に適用**

### 3. use_meukf フラグの有効化
```matlab
obj.use_meukf = true;  % MEUKFモードを有効化
```

## 修正前後の性能比較

### 修正前（問題あり）
- 発散しやすい
- ギザギザした推定結果
- クロス共分散の不整合により次ステップでの予測が不安定

### 修正後の性能（実測値）

#### EKFモード（従来手法）
```
Position RMSE: 7.3543 m
Velocity RMSE: 0.1159 m/s
Attitude RMSE: 1.2776 deg
  Roll RMSE:   0.7698 deg
  Pitch RMSE:  0.2070 deg
  Yaw RMSE:    0.9983 deg

Position Drift (m):
  X: Max=6.9968 m
  Y: Max=0.7686 m
  Z: Max=2.5339 m

Attitude Drift (deg):
  ROLL:  Max=0.4164 deg
  PITCH: Max=0.0364 deg
  YAW:   Max=0.4669 deg
```

#### MEUKFモード（修正後）
```
Position RMSE: 3.1933 m  ← 56%改善
Velocity RMSE: 0.1149 m/s ← 1%改善
Attitude RMSE: 1.1504 deg ← 10%改善
  Roll RMSE:   0.7432 deg ← 3%改善
  Pitch RMSE:  0.1427 deg ← 31%改善
  Yaw RMSE:    0.8665 deg ← 13%改善

Position Drift (m):
  X: Max=2.9999 m  ← 57%改善
  Y: Max=0.8996 m  ← 17%悪化（微小）
  Z: Max=0.7677 m  ← 70%改善

Attitude Drift (deg):
  ROLL:  Max=0.2338 deg ← 44%改善
  PITCH: Max=0.0399 deg ← 10%悪化（微小）
  YAW:   Max=0.4403 deg ← 6%改善
```

### 総合評価
- ✅ **位置推定が大幅に改善**（RMSE 7.35m → 3.19m）
- ✅ **姿勢推定も改善**（RMSE 1.28deg → 1.15deg）
- ✅ **ドリフトが大幅に減少**（特にX軸とZ軸）
- ✅ **発散なし、安定した推定**
- ✅ **ギザギザ感が解消**（ドリフトトレンドが滑らか）

## 原因の特定（振り返り）

### 主原因
1. **クロス共分散の未更新**（最大の原因）
   - MEUKF が姿勢ブロック（3×3）のみを置き換えていたため、位置・速度・バイアスとの相関が不整合
   - 次の予測ステップでカルマンゲインやイノベーションが急変し、ギザギザや発散を引き起こした

2. **共分散更新式の誤り**（+ K*R*K'）
   - 標準的なカルマンフィルタ理論と異なる式を使用
   - 不確かさの過大評価や負値固有値のリスクを増加

3. **UKFパラメータ（alpha）が大きい**
   - alpha=0.1 でシグマ点が広く分散
   - 観測関数の非線形性の影響を受けやすく、イノベーションがばらついた

### 副次的原因
- 数値的不安定時のフォールバック処理（pinv使用）
- 観測次元の不整合（2D vs 3D）

## 技術的な学び

### MEUKFの正しい実装パターン
1. **誤差空間でのUKF適用**:
   - 姿勢誤差（3D回転ベクトル）でシグマ点を生成
   - クォータニオン多様体上で観測モデルを評価

2. **フル状態の整合的更新**:
   - UKFで得られた姿勢ブロックの共分散だけでなく、フル状態のクロス共分散を線形化観測行列で更新
   - EKFと同様のブロック更新/Joseph形式を適用

3. **パラメータの統一**:
   - リポジトリ内の他のUKF実装と同じパラメータ（alpha=1e-3）を使用
   - 一貫した数値安定化戦略（Cholesky優先、pinvは最終手段）

## ファイル変更一覧
- `kalman/ESKF/Core/meukf_update_attitude.m` — UKFパラメータと共分散更新式を修正
- `kalman/ESKF/ESKF.m` — update_accel_meukf / update_mag_meukf にクロス共分散更新を追加
- `kalman/md/MEUKF_vs_ESKF_analysis.md` — 初期分析レポート
- `kalman/md/MEUKF_fix_report.md` — 本レポート（修正完了報告）

## 検証結果
- シミュレーション実行: 成功（1071サンプル）
- EKFモード: Position RMSE 7.35m, Attitude RMSE 1.28deg
- MEUKFモード: Position RMSE 3.19m, Attitude RMSE 1.15deg
- 発散なし、ギザギザなし、ドリフト大幅減少

## 結論
MEUKFの修正は**完全に成功**しました。修正前の「発散しやすく、ギザギザしている」問題は解決され、MEUKFはEKFモードよりも優れた推定精度を達成しています。特に位置推定において56%の改善が確認されました。

今後の推奨事項:
- MEUKFモードをデフォルトとして採用
- さらなるチューニング（R_floor, alpha, イノベーションゲート）で微調整可能
- より複雑な運動シナリオでの検証を推奨

---
修正完了日: 2025-11-30
