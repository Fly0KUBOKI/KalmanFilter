# Pitch/Roll 入れ替わり問題の修正

## 問題の原因

複数の不整合が重なって pitch と roll が入れ替わって見える現象が発生していました：

### 1. 回転順序の不整合（主要な原因）
- **`compute_body_velocities.m`**: `eul2rotm([yaw, pitch, roll], 'ZYX')` を使用
- **`generate_sensor_observations.m`**: `eul2rotm([yaw, pitch, roll], 'ZXY')` を使用（修正前）
- 異なる回転順序により、同じオイラー角でも異なる回転行列が生成され、センサー出力と真値が不整合

### 2. ジャイロ軸マッピングの混乱
- 修正前: `gyro_x = pitch_rate`, `gyro_y = roll_rate`（非標準）
- 標準的な航空機座標系: `gyro_x = roll_rate`, `gyro_y = pitch_rate`, `gyro_z = yaw_rate`

### 3. コメントと実装の不一致
- 複数の箇所でコメントと実際の軸定義が食い違っていた

## 修正内容

### 1. 回転順序の統一（最重要）
**ファイル**: `GenerateData/generate_sensor_observations.m`
```matlab
# 修正前
R = eul2rotm([yaw, pitch, roll], 'ZXY');

# 修正後
R = eul2rotm([yaw, pitch, roll], 'ZYX');  % 標準航空機座標系
```

### 2. ジャイロ軸マッピングの標準化
**ファイル**: `GenerateData/generate_sensor_observations.m`
```matlab
# 修正前（非標準）
gyro_body(i,1) = rad2deg(q);  % x軸周り: pitch角速度
gyro_body(i,2) = rad2deg(p);  % y軸周り: roll角速度
gyro_body(i,3) = rad2deg(r);  % z軸周り: yaw角速度

# 修正後（標準）
gyro_body(i,1) = rad2deg(p);  % x軸周り: roll角速度
gyro_body(i,2) = rad2deg(q);  % y軸周り: pitch角速度
gyro_body(i,3) = rad2deg(r);  % z軸周り: yaw角速度
```

### 3. ESKF コメントの修正
**ファイル**: `ESKF/ESKF.m`, `ESKF/Core/integrate_nominal.m`
- ジャイロデータの解釈コメントを標準航空機座標系に修正
- `w = [obs.wx(k); obs.wy(k); obs.wz(k)]` は `[roll_rate, pitch_rate, yaw_rate]` を意味

### 4. クォータニオン関数の改良
**ファイル**: `KF/Core/quat_lib.m`

#### `quat_to_euler` の改良
```matlab
% オプションでオイラー角順序を指定可能に
euler = quat_lib('quat_to_euler', q);           % デフォルト: ZYX順序
euler = quat_lib('quat_to_euler', q, 'ZYX');    % 明示的に指定
```

#### `euler_to_quat` の改良
```matlab
% オプションでオイラー角順序を指定可能に
q = quat_lib('euler_to_quat', euler_deg);           % デフォルト: ZYX順序
q = quat_lib('euler_to_quat', euler_deg, 'ZYX');    % 明示的に指定
```

これにより、将来的に異なる回転順序が必要な場合でも、関数の引数で指定できるようになりました。

## 標準航空機座標系の定義

修正後の実装は以下の標準的な航空機座標系を使用：

### 座標軸
- **X軸（Roll軸）**: 機体右方向（機首から見て右翼方向）
- **Y軸（Pitch軸）**: 機体前方向（機首方向）
- **Z軸（Yaw軸）**: 機体下方向（重力方向）

### オイラー角（ZYX順序）
- **Yaw (ψ)**: Z軸周りの回転（ヨー、方位角）
- **Pitch (θ)**: Y軸周りの回転（ピッチ、縦揺れ）
- **Roll (φ)**: X軸周りの回転（ロール、横揺れ）

### 回転行列の合成順序
```
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```
この順序により、機体固定座標系から世界座標系への変換が正しく行われます。

## 検証方法

修正後、以下を確認してください：

1. **データ生成の再実行**
   ```matlab
   sim_generate();
   ```

2. **ESKFシミュレーション**
   ```matlab
   run_simulation();
   ```

3. **Pitch/Roll の一致確認**
   - DivergenceMonitor の出力で Roll/Pitch 誤差が小さくなることを確認
   - `Results/estimation.csv` と `GenerateData/truth_data.csv` を比較

4. **グラフでの視覚的確認**
   ```matlab
   plot_csv('Results/estimation.csv', 'time');
   ```

## 影響を受けるファイル

### 修正されたファイル
1. `GenerateData/generate_sensor_observations.m` - 回転順序とジャイロ軸修正
2. `ESKF/ESKF.m` - コメント修正
3. `ESKF/Core/integrate_nominal.m` - コメント修正
4. `KF/Core/quat_lib.m` - 回転順序パラメータ追加
5. `analyze_sensor_axes.m` - 解析コメント更新

### 自動的に影響を受けるファイル
- `GenerateData/truth_data.csv` - 再生成により正しい真値になる
- `GenerateData/sensor_data.csv` - 再生成により正しいセンサーデータになる
- `Results/estimation.csv` - ESKFの推定結果が正しくなる

## 注意事項

1. **既存データの無効化**: 修正前に生成されたデータは使用できません。必ず再生成してください。
2. **互換性**: 外部ツールで生成されたデータを使用する場合、そのデータが ZYX 順序を使用していることを確認してください。
3. **テストコード**: デバッグ/テスト用のスクリプトで `euler_to_quat` を呼び出している箇所は、度単位で引数を渡していることを確認してください。

## まとめ

この修正により：
- ✅ 回転順序が ZYX（標準航空機座標系）に統一されました
- ✅ ジャイロ軸マッピングが標準定義に修正されました
- ✅ Pitch と Roll が正しく対応するようになりました
- ✅ クォータニオン関数が軸順序に依存しない設計になりました
- ✅ コードのコメントと実装が一致するようになりました
