# パルス検知機能

## 概要

ESKFフィルタに、**全ての推定値**（位置、速度、姿勢）における急激な変化（パルス）を自動検知してログに記録する機能を追加しました。

## 背景

- 加速度計のみを使用した姿勢推定において、特定のタイミングで小さな急変（パルス）が観測されました
- データ生成のたびにパルスの発生タイミングが変わるため、手動での調査では全てのパルスを追跡することが困難
- 自動検知システムにより、全シミュレーション実行時にパルスを記録して統計的な分析を可能にします

## 実装内容

### 1. ESKFクラスへの追加プロパティ

`ESKF/ESKF.m`のpropertiesセクションに以下を追加：

```matlab
pulse_detection_enabled  % パルス検知の有効/無効
pulse_threshold          % 検知閾値（姿勢:度、位置:m、速度:m/s）
pulse_log               % パルスログ（struct配列）
prev_position           % 前ステップの位置
prev_velocity           % 前ステップの速度
prev_euler              % 前ステップのEuler角
```

### 2. パルス検知ロジック

`detectPulse`メソッドで、以下の処理を実行：

1. 現在の位置、速度、姿勢と前ステップの値の差分を計算
2. いずれかの変化が閾値（デフォルト0.05）を超えた場合、パルスとして記録
3. ログには以下の情報を保存：
   - ステップ番号、時刻
   - パルスタイプ（attitude / position / velocity / 複合）
   - 全9軸の変化量（roll, pitch, yaw, px, py, pz, vx, vy, vz）

### 3. 検知対象

| 種類 | 対象軸 | 単位 | 説明 |
|------|--------|------|------|
| **姿勢 (attitude)** | Roll, Pitch, Yaw | 度 | Euler角の急変 |
| **位置 (position)** | X, Y, Z | m | 位置推定の急変 |
| **速度 (velocity)** | VX, VY, VZ | m/s | 速度推定の急変 |

### 4. ログ取得メソッド

```matlab
pulse_log = kf.getPulseLog();
```

パルス検知ログを取得します。

### 5. run_simulation.mへの統合

フィルタリング完了後、パルスログを自動的に取得して表示・保存：

```matlab
pulse_log = kf.getPulseLog();
if ~isempty(pulse_log)
    fprintf('\n★ 検知されたパルス: %d 箇所\n', length(pulse_log));
    pulse_table = struct2table(pulse_log);
    writetable(pulse_table, fullfile(outDir, 'pulse_log.csv'));
end
```

## 使用方法

### 基本的な使用

通常通り`run_simulation`を実行するだけで、パルス検知が自動的に行われます：

```matlab
run_simulation
```

パルスが検知された場合：
- コンソールに各パルスの情報が表示されます
- `Results/pulse_log.csv`にログが保存されます

### コンソール出力例

```
★ パルス検知 [Step 21004, 52.508s] タイプ=attitude
   姿勢: Roll=0.084° Pitch=0.012° Yaw=0.156°

★ パルス検知 [Step 27076, 67.688s] タイプ=attitude+velocity
   姿勢: Roll=0.042° Pitch=0.008° Yaw=0.023°
   速度: VX=0.065m/s VY=0.012m/s VZ=0.003m/s
```

### 閾値の調整

ESKFクラスのコンストラクタで閾値を変更できます（`ESKF.m`の207行目付近）：

```matlab
obj.pulse_threshold = 0.05;  % 例：0.05（姿勢:度、位置:m、速度:m/s）
```

推奨値：
- **0.05** (デフォルト): 通常使用。小さな変化も検知
- **0.10**: 中感度。顕著なパルスのみ検知
- **0.20**: 低感度。大きなパルスのみ検知

注意：同じ閾値が姿勢（度）、位置（m）、速度（m/s）の全てに適用されます。

### パルス検知の無効化

必要に応じて、パルス検知を無効化できます：

```matlab
obj.pulse_detection_enabled = false;  % コンストラクタ内
```

## 出力ファイル

### pulse_log.csv

| カラム | 説明 |
|--------|------|
| step | パルス発生ステップ番号 |
| time | パルス発生時刻（秒） |
| type | パルスタイプ（attitude / position / velocity / 複合） |
| roll_change | Roll変化量（度） |
| pitch_change | Pitch変化量（度） |
| yaw_change | Yaw変化量（度） |
| px_change | X位置変化量（m） |
| py_change | Y位置変化量（m） |
| pz_change | Z位置変化量（m） |
| vx_change | VX速度変化量（m/s） |
| vy_change | VY速度変化量（m/s） |
| vz_change | VZ速度変化量（m/s） |

## テスト

短時間のテストには`test_pulse_detection.m`を使用できます：

```matlab
test_pulse_detection
```

このスクリプトは最初の5000ステップのみを実行し、パルスログを表示します。

## パフォーマンス

- パルス検知処理のオーバーヘッドは非常に小さい（ステップあたり数マイクロ秒）
- 36,001ステップの完全なシミュレーションでも、実行時間への影響はほぼありません

## トラブルシューティング

### パルスが検知されない場合

1. 閾値が高すぎる可能性があります。`pulse_threshold`を下げてみてください
2. データ生成時のランダム性により、パルスが発生しない場合もあります

### パルスが多すぎる場合

1. 閾値が低すぎる可能性があります。`pulse_threshold`を上げてみてください
2. 通常の変化もパルスとして検出されている可能性があります
   - 姿勢のみを監視したい場合：閾値を0.10°程度に上げる
   - Yaw角の大きな変化が気になる場合：Yawは磁気計更新のタイミングで変化するため正常です

## 実装の詳細

### パルス検知のタイミング

パルス検知は`updateFilter`の最後に実行されます：

```matlab
% updateFilter関数内
obj.predict(a, w);
obj.updateAccel(a);
obj.updateMag(...);
obj.updateBaro(...);
obj.updateGPS(...);
obj.detectPulse(k, obs.time(k));  % ← 全ての更新後に検知
```

これにより、全てのセンサー更新が反映された最終的な推定値の変化を捉えます。

### 検知される変化の種類

1. **姿勢パルス（attitude）**
   - Roll/Pitch: 加速度計による直接計算の数値誤差
   - Yaw: 磁気計更新による補正ジャンプ

2. **位置パルス（position）**
   - GPS更新による補正ジャンプ
   - 予測ステップでの積分誤差

3. **速度パルス（velocity）**
   - 加速度積分誤差
   - GPS速度推定による補正

## 今後の拡張

- [ ] 軸ごとに異なる閾値を設定（姿勢、位置、速度で別々の閾値）
- [ ] パルスの統計分析機能（発生頻度、平均変化量など）
- [ ] パルス発生時の詳細なデバッグ情報保存
- [ ] 連続するパルスのグループ化

## 関連ファイル

- `ESKF/ESKF.m`: メインフィルタクラス（パルス検知実装）
- `run_simulation.m`: シミュレーション実行スクリプト（パルスログ保存）
- `test_pulse_detection.m`: パルス検知のテストスクリプト
- `Results/pulse_log.csv`: 出力されるパルスログ
