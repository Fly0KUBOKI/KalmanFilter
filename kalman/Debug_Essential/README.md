# Essential Debug Tools

このフォルダには最小限の必須デバッグツールが含まれています。

## ツール一覧

### 1. quick_check.m
**目的**: 高速な健全性チェック

**使用タイミング**:
- コード変更後の即座の動作確認
- CI/CD パイプラインでの自動テスト

**使い方**:
```matlab
status = quick_check()
```

**出力**:
- status: 0=正常, 1=警告, 2=エラー
- コンソールに PASS/WARN/FAIL 表示
- 各項目の閾値チェック結果

---

### 2. compare_results.m
**目的**: 推定結果と真値の詳細比較

**使用タイミング**: 
- シミュレーション実行後に結果の精度を確認したい時
- pitch/roll の整合性を検証したい時

**使い方**:
```matlab
compare_results()
```

**出力**:
- Position, Velocity, Attitude (roll/pitch/yaw) の RMS エラー
- 最大エラー値とその発生時刻
- コンソールに簡潔なサマリー表示

---

### 3. plot_errors.m
**目的**: エラーの時系列可視化

**使用タイミング**:
- エラーの推移パターンを視覚的に確認したい時
- 発散が発生した時刻を特定したい時

**使い方**:
```matlab
plot_errors()
```

**出力**:
- 3つのサブプロット: Position Error, Velocity Error, Attitude Error
- 時間軸に対するエラーの変化グラフ

---

### 4. plot_trajectory.m
**目的**: 3D軌跡の可視化

**使用タイミング**:
- 推定軌跡と真値軌跡の差を視覚的に確認したい時
- 軌跡の全体像を把握したい時

**使い方**:
```matlab
plot_trajectory()
```

**出力**:
- 3D空間での軌跡プロット (推定 vs 真値)
- スタート/ゴール地点の表示
- Position RMS/Max/Mean エラー統計

---

## ファイル構造要件

スクリプト実行前に以下のファイルが必要です:
- `Results/estimation.csv` - ESKF推定結果
- `GenerateData/truth_data.csv` - 真値データ

---

## 使用例

### 基本的なワークフロー
```matlab
% 1. データ生成
sim_generate();

% 2. ESKF実行
run_simulation();

% 3. 結果確認
quick_check();      % 高速チェック (数値判定)
compare_results();  % 詳細な統計
plot_errors();      % エラーのグラフ表示
plot_trajectory();  % 軌跡のグラフ表示
```

### CI/CDでの使用
```matlab
status = quick_check();
if status ~= 0
    error('Simulation failed health check');
end
```

---

## 設計思想

- **最小限**: 4つのツールのみ、各100行以下
- **自己完結**: 外部依存なし、標準MATLAB関数のみ使用
- **高速**: 数秒以内に実行完了
- **明確な出力**: 合否判定が一目でわかる

---

## 削除された機能

以下の機能は不要と判断し削除されました:
- 詳細なデバッグダンプ (saveDetailedLog)
- ステップバイステップのトレース (callDebug)
- 緊急停止機能 (emergencyStop)
- 冗長な進捗表示 (1000ステップごとの出力)
- 50+個の旧Debug/フォルダのスクリプト
- 全てのtest_*.m (kalman/ルート)
- 全てのdebug_*.m (kalman/ルート)
- analyze_divergence.m, analyze_roll_divergence.m, verify_fix.m
