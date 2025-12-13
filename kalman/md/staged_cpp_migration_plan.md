# 段階的C++移行計画

## 現状 (2025年12月13日)

### ✅ 完了
- SensorDataBuffer クラス作成・テスト完了
- プロット生成エラー修正（バッチ実行安定化）
- unified_filter.hpp/cpp 基本実装

### 🔄 実行中
- batch_10sets ベースライン確認

## フェーズ1: センサーデータ変更検知の統合 (1-2日)

### 目標
MATLABレベルでセンサーデータの変更を検知し、C++側への不要な呼び出しを削減

### 実装
1. **ESKF.mにSensorDataBuffer統合**
   ```matlab
   % ESKF.m constructor
   obj.sensor_buffer = SensorDataBuffer();
   
   % predict/update内
   if obj.sensor_buffer.has_accel_changed(a)
       % C++ MEX呼び出し
   end
   ```

2. **run_simulation.m修正**
   - センサー更新頻度に応じてデータ複製
   - GPS: 40サンプルに1回 → 同じ値を40回生成

3. **検証**
   - batch_10で精度維持確認
   - MEX呼び出し回数のログ

## フェーズ2: 統合MEXインターフェース (3-5日)

### 目標
全センサーを1回のMEX呼び出しで処理

### 実装
1. **mex_unified_filter.cpp作成**
   - FilterInput/OutputをMATLAB構造体にマッピング
   - mex_meukf_step_v2と並行稼働

2. **ESKF.mにオプション追加**
   ```matlab
   % Constructor option
   obj.use_unified_mex = false; % デフォルトは既存MEX
   
   if obj.use_unified_mex
       output = mex_unified_filter(input);
   else
       % 既存のcall_cpp_update_impl
   end
   ```

3. **ビルド・テスト**
   - build_mex.m更新
   - batch_10で両実装を比較

## フェーズ3: C++側フィルタ統合 (1週間)

### 目標
SensorFilterLib, NoiseEstimator, DivergenceGuardをC++に完全移行

### 実装
1. **unified_filter.cpp完成**
   - センサーフィルタリング
   - 適応的ノイズ推定
   - 発散検出・回復

2. **MATLAB側の簡素化**
   - SensorFilterLib.m → C++へ
   - NoiseEstimatorLib.m → C++へ
   - DivergenceGuard.m → C++へ

3. **検証**
   - 精度: Position RMSE < 5.0m維持
   - 性能: 処理時間50%削減目標

## フェーズ4: MATLAB薄型ラッパー化 (3-5日)

### 目標
ESKF.mを50行程度のシンプルなラッパーに

### 実装
1. **ESKF_Simple.m作成**
   ```matlab
   classdef ESKF_Simple < handle
       properties
           mex_state  % C++側で状態管理
       end
       
       methods
           function obj = ESKF_Simple(obs, static_time, dt)
               % 初期化のみ
               obj.mex_state = mex_unified_filter('init', ...);
           end
           
           function step(obj, sensor_data)
               % 単純なMEX呼び出し
               obj.mex_state = mex_unified_filter('step', ...);
           end
       end
   end
   ```

2. **run_simulation.m修正**
   - ESKF → ESKF_Simple切り替えオプション

3. **後方互換性**
   - 既存ESKFは残す（deprecation warning）

## フェーズ5: 最終最適化 (1週間)

### 目標
パフォーマンスチューニングと文書化

### 実装
1. **バッチ処理最適化**
   - ループ内のメモリアロケーション削減
   - C++側でのベクトル化

2. **診断機能強化**
   - デバッグ情報の充実
   - パフォーマンスプロファイリング

3. **文書更新**
   - .github/copilot-instructions.md
   - md/cpp_complete_migration.md

## 成功基準

### 機能要件
- [ ] Position RMSE < 5.0m (既存と同等)
- [ ] Attitude RMSE: Roll/Pitch < 1.0°, Yaw < 2.0°
- [ ] NaN/Inf 発生なし
- [ ] 10回連続成功 (batch_10)

### 性能要件
- [ ] 処理時間 50%削減
- [ ] メモリ使用量 30%削減
- [ ] MEX呼び出しオーバーヘッド最小化

### 保守性
- [ ] コード行数: MATLAB 600行以下
- [ ] C++ 単体テスト 80%カバレッジ
- [ ] 文書化完了

## リスク管理

### 高リスク
- **精度劣化**: フェーズごとにbatch_10で検証
- **性能低下**: プロファイリング必須

### 対策
- 既存実装を常に保持（ロールバック可能）
- A/Bテスト機能（両実装を同時実行して比較）

## 次のステップ

1. batch_10結果確認
2. SensorDataBuffer統合 (フェーズ1開始)
3. 週次進捗レビュー
