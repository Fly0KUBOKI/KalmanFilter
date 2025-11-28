````markdown
# 加速度更新アルゴリズムの180度反転バグ修正記録

**日付**: 2025年11月17日  
**問題**: ジャイロなし加速度計のみの姿勢推定で180度反転が発生  
**結果**: ✅ 修正完了 - RMS誤差 0.21° を達成

---

## 問題の概要

ESKFの加速度更新において、ジャイロを無効化して加速度計のみで姿勢推定を行うと、Roll角が180度反転する問題が発生していた。

### 症状
- Roll RMS誤差: **166.79°**
- 88.5%のタイムステップで約±180°の誤差
- ステップ231で最初の反転が発生
- ステップ212から指数関数的に劣化開始

---

## 根本原因の特定

### 1. 直接計算との比較

**直接atan2計算**を実装してテスト：
```matlab
roll_measured = atan2(ay, az);
pitch_measured = atan2(-ax, sqrt(ay^2 + az^2));
```

**結果**:
- Roll RMS: **0.2141°** ← 正常に動作
- Pitch RMS: **0.2781°**
- 180度反転: **0回**

→ **データには問題なし。アルゴリズムのバグと確定**

### 2. 発散開始ポイントの特定

詳細追跡により発散の時系列を特定：

| ステップ | Roll誤差 | cos(angle) | 状態 |
|---------|---------|-----------|------|
| 1-201 | 0.00° | 1.000 | ✅ 正常 |
| 202 | -0.00° | 1.000 | 加速度変化開始 |
| 209 | -0.13° | 0.99997 | 誤差蓄積開始 |
| 210 | -0.26° | 0.99999 | 倍増 |
| 211 | -0.51° | 0.99996 | 倍増 |
| 212 | -1.03° | 0.99984 | 倍増 |
| 213 | -2.06° | 0.99935 | 倍増 |
| 214 | -4.12° | 0.99741 | 倍増 |
| 215 | -8.24° | 0.98966 | 倍増 |
| 216 | -16.49° | 0.95887 | 補正制限到達 |
| 217-223 | -26°→-86° | 正→0 | 毎ステップ-10° |
| 224 | -96.49° | **-0.113** | cos(angle)が負に！ |
| 225+ | -106°→-176° | 負 | 180度反転状態 |

**パターン**: 指数関数的劣化（倍々ゲーム）→ 正のフィードバックループ

### 3. 正のフィードバックループの構造

旧実装の更新式:
```matlab
R = quat_to_rotm(q);                    % 現在の姿勢から回転行列
a_expected = R' * specific_force_world;  % 期待加速度を計算 ← ★問題
a_meas_unit = a_meas / norm(a_meas);
a_exp_unit = a_expected / norm(a_expected);
rotation_axis = cross(a_exp_unit, a_meas_unit);
angle = atan2(norm(rotation_axis), dot(a_exp_unit, a_meas_unit));
dq = small_angle_quat(rotation_axis * angle);
q = quatmultiply(q, dq);                % クォータニオン更新
```

**問題点**:
1. Rollに誤差がある
2. `R'`に誤差が含まれる
3. `a_expected`の計算が誤る（特にY成分の符号反転）
4. `cross(a_exp, a_meas)`が逆方向の補正を計算
5. Rollがさらに誤る → **悪循環**

#### 具体例（ステップ213の場合）

```
現在のRoll: -2.06°
a_meas:     [-0.000003, 0.004138, 9.810000]  ← ay > 0
a_expected: [ 0.000076, -0.348700, 9.803802] ← ay < 0 (符号逆！)

cross(a_exp, a_meas) = [-0.035967, -0.000008, -0.000000]
→ X軸周りに-2.06°回転の補正
→ Roll = -2.06° + (-2.06°) = -4.12°
→ 次のステップでさらに悪化
```

---

## 修正方法

### アプローチ

**誤った期待値に基づく反復補正**を止め、**加速度計から直接Roll/Pitchを計算**する方式に変更。

### 修正後のアルゴリズム

```matlab
function q_new = update_with_direct_angles(q, a_meas)
    % 加速度の妥当性チェック
    a_norm = norm(a_meas);
    if a_norm < 0.1 || abs(a_norm - 9.81) > 3.0
        q_new = q;
        return;
    end
    
    % 現在のYawを保持（加速度計では観測不可能）
    euler_current = quat_to_euler(q);
    yaw_current = euler_current(3);
    
    % 加速度から直接Roll/Pitchを計算
    ax = a_meas(1);
    ay = a_meas(2);
    az = a_meas(3);
    
    roll_measured = atan2(ay, az);
    pitch_measured = atan2(-ax, sqrt(ay^2 + az^2));
    
    % 新しいクォータニオンを生成
    q_new = euler_to_quat([roll_measured; pitch_measured; yaw_current]);
end
```

### 重要なポイント

1. **反復計算を排除**: 誤差が蓄積する余地をなくす
2. **直接計算**: atan2により一意な解を得る
3. **Yaw保持**: 加速度計では観測不可能なYawは保持
4. **正のフィードバック回避**: 現在の姿勢推定に依存しない

---

## 修正結果

### 性能比較

| 指標 | 修正前 | 修正後 | 改善率 |
|-----|--------|--------|--------|
| Roll RMS誤差 | 166.79° | **0.2141°** | **99.87%改善** |
| Pitch RMS誤差 | 9.09° | **0.2781°** | **96.94%改善** |
| Roll最大誤差 | 179.998° | **1.2855°** | **99.29%改善** |
| 180度反転回数 | 1770/2000 (88.5%) | **0/36001 (0%)** | **完全解消** |

### テスト条件

- データセット: 36,001ステップ（90秒）
- サンプリング周波数: 400 Hz
- シミュレーション: 円運動（等速）
- センサー: 加速度計のみ（ジャイロ無効）

---

## 実装への反映

### 修正対象ファイル

1. **`ESKF/ESKF.m`** - `updateAccel`関数を修正

### 修正内容

`updateAccel`関数を以下のように変更（行288-350）：

**Before（バグあり）**:
```matlab
% 現在の推定姿勢から期待される加速度を計算
R_current = quat_to_rotm(q);
specific_force_world = [0; 0; 9.81];
a_expected_body = R_current' * specific_force_world;
a_expected_unit = a_expected_body / norm(a_expected_body);

% 最小回転を計算
cross_vec = cross(a_expected_unit, a_unit);
sin_angle = norm(cross_vec);
cos_angle = dot(a_expected_unit, a_unit);
angle = atan2(sin_angle, cos_angle);
axis = cross_vec / sin_angle;
dtheta = axis * angle;

% クォータニオン更新
dq = small_angle_quat(dtheta);
q = quatmultiply(q, dq);
```

**After（修正版）**:
```matlab
% 現在のYawを保持（加速度計では観測不可能）
euler_current = quat_to_euler(q);
yaw_current = euler_current(3);

% 加速度から直接Roll/Pitchを計算
ax = a_corrected(1);
ay = a_corrected(2);
az = a_corrected(3);

roll_measured = atan2(ay, az);
pitch_measured = atan2(-ax, sqrt(ay^2 + az^2));

% 新しいクォータニオンを生成（Yawは保持）
q = euler_to_quat([roll_measured; pitch_measured; yaw_current]);
```

### 実装上の注意点

1. **適用範囲**: この修正は**ジャイロ無効時の加速度のみによる姿勢推定**に適用
2. **前提条件**: 加速度計が主に重力を測定していること（静止または低加速度運動）
3. **制限事項**: 高速運動時は遠心力の影響で精度が低下する可能性あり
4. **Yaw**: 加速度計では観測不可能なため、現在値を保持（磁気計やGPSで更新）

### 修正結果

✅ **修正完了** (2025/11/17)
- ESKF/ESKF.m の updateAccel 関数を直接計算方式に変更
- 180度反転問題を完全解消
- コード内に詳細なコメントと修正履歴を追加

---

## 検証

### テストプログラム

1. `test_direct_angle_calculation.m` - 直接計算の妥当性確認
2. `debug_quaternion_update_convergence.m` - 初期値依存性の確認
3. `find_exact_divergence.m` - 発散開始ポイントの特定
4. `analyze_positive_feedback.m` - フィードバックループの解析
5. `test_corrected_accel_update.m` - 修正版の性能検証

### 結論

✅ **修正アルゴリズムは全テストケースで正常動作**
- Roll/Pitch推定精度: 0.2°台（サブdegree精度）
- 180度反転: 完全に解消
- 正のフィードバックループ: 根本的に排除

---

## 学んだ教訓

1. **反復補正の危険性**: 誤った推定値に基づく補正は誤差を増幅する
2. **直接計算の優位性**: 可能な限り直接計算を使用すべき
3. **初期仮説の再検証**: 「物理的限界」と思われた問題も実装バグの可能性
4. **段階的デバッグ**: 異なる手法の比較により根本原因を特定

---

## 関連ファイル

- テスト結果: `Results/corrected_accel_update_test.png`
- 実装: `ESKF/Core/AttitudeUpdate.m`
- この記録: `ACCEL_UPDATE_180DEG_FIX.md`

````