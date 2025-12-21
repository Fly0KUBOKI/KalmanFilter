# GitHub Copilot 指示 — KalmanFilter

## 概要
MATLAB（制御・I/O・可視化）+ C++ MEX（高速数値処理）のハイブリッド実装によるカルマンフィルタプロジェクト。  
**主な目的**: C++ コアを修正 → MEX ビルド → MATLAB 側と数値パリティ検証 の流れを安全・迅速に実行する。

## アーキテクチャの「なぜ」
- **MATLAB層** (`kalman/run_simulation.m`): 実験設計・結果収集・比較用（1回のシミュレーション: ~50秒）
- **C++ MEX層** (`kalman/cpp/` 以下): 計算ボトルネック（予測・更新×11,600ステップ）をC++で実装
- **状態ベクトル** = 15次元 `[p(3), v(3), q(4), ba(3), bg(3)]`
  - `q = [w, x, y, z]`（スカラー先頭）のクォータニオン形式固定
  - C++ では `float64` で保持（PHASE 4 で精度損失問題特定済み）

## 必須ビルド・テストワークフロー

### C++修正→検証の3ステップ
```matlab
% 1. ビルド（特定ターゲットのみ選択可）
cd kalman/cpp/build
build_mex()                    % 全体 or build_mex({'mex_meukf_step_v2'})
clear mex                      % MATLAB キャッシュ必須クリア

% 2. 単一実行（データ再生成スキップ）
cd ../../..
run_simulation(42, true)       % seed=42, skip_data_gen=true
%  ↓ または バッチ実行（10組）
run_batch_10sets()             % Results/に複数CSV出力

% 3. 差分チェック
compare_mex_matlab_detailed    % CSV差分表示
```

### よくある検証パターン
- **小さな修正**: `build_mex({'mex_quaternion_lib'})` で単体テスト
- **センサー関連**: `mex_meukf_step_v2` を中心に確認（70%の呼び出しシェア）
- **精度問題**: `Results/estimation_*.csv` で行ごと差分確認

## プロジェクト固有の「絶対守るルール」

| ルール | 理由・補足 |
|--------|-----------|
| MEX バイナリは `bin/` フォルダ内 — 直接編集禁止 | 成果物フォルダ。必ずソース修正→`build_mex()` |
| MEX 更新後は `clear mex` 実行 | キャッシュが古い .mexw64 を使い続ける落とし穴 |
| 共分散は対称性保持 `P = (P + P')/2` | 数値丸め誤差で非対称化するため |
| C++ 側がクォータニオン正規化 | MATLAB で二重正規化するな（ノーマライズ済みを重ねると精度低下） |
| 状態フィールド順序厳格: p,v,q,ba,bg,P | MEX インターフェースがこの順で読み込む |

## 主要ファイル・エントリポイント

| ファイル | 役割 | 補足 |
|---------|------|------|
| `run_simulation.m` | MATLAB メイン (seed, skip_data_gen引数) | 1回: ~50秒 |
| `run_batch_10sets.m` | 10セット自動実行 + CSV比較 | 検証の標準手段 |
| `ESKF/@ESKF/ESKF.m` | フィルタクラス（predict, sensor_updates, zupt） | MEX 呼び出しハブ |
| `sensor_updates.m` | センサー測定値→MEX呼び出し | struct 構築・フラグ設定 |
| `config_params.m` | シミュレーションパラメータ（ノイズ、dt=0.0025） | PHASE 5 で変更可能性大 |
| `cpp/build/build_mex.m` | ビルドスクリプト | `mex -setup C++` 要確認 |
| `Results/estimation_*.csv` | MATLAB/MEX 出力結果 | 差分分析の基本データ |

## MEX インターフェース構造（MATLAB↔C++）

### 典型的な呼び出し例 (`sensor_updates.m` より)
```matlab
state = struct('p', p, 'v', v, 'q', q, 'ba', ba, 'bg', bg, 'P', P);
sensor_data = struct('accel', accel, 'gyro', gyro, ..., 
                     'update_accel', true, 'update_gyro', false, ...);
mex_params = struct('g', g, 'noise_accel', R_accel, 'alpha', 0.1, ...);

new_state = mex_meukf_step_v2(state, sensor_data, mex_params);
% 戻り値: state（更新後）と debug_info（オプション）
```

### 状態・センサー・パラメータ struct の仕様
- **state**: `p,v,q,ba,bg,P` (P=15×15行列)
- **sensor_data**: `accel,gyro,mag,gps_pos,alt_baro,dt, update_*フラグ,prev_*` (変更検知用)
- **mex_params**: `g,mag_ref,noise_*,alpha,beta,kappa` (UKF/MEUKF ハイパーパラメータ)

## デバッグ・トラブルシューティング

### ビルド失敗
- **症状**: `build_log.txt` に未定義シンボル、コンパイラエラー
- **チェック**: `mex -setup C++` で MSVC/MinGW が正しく設定されているか確認
- **対応**: `cpp/build/build_result.txt` に詳細ログ → ファイル依存関係確認

### MEX 実行で NaN/推定値欠落
- **症状**: `Results/` に CSV が生成されない、または値が全て NaN
- **原因**: 多くは struct フィールド不一致か、初期状態が異常
- **確認**: `sensor_updates.m` の状態フィールド順序と struct キー名を再確認

### MATLAB vs MEX 数値差が大きい（>1%）
- **原因**: C++ 側が `float32` で計算している（PHASE 4 で検出）
- **対応**: C++ ソースを `float64` に変更 → `build_mex()` → 再検証
- **詳細**: `cpp/markdown/COMPLETION_REPORT.md` の "32-bit float 精度損失" を参照

### センサー更新順序・頻度の制御
- **場所**: `run_simulation.m` の メインループ内で `ESKF.predict()` / `ESKF.sensor_updates()` を条件付け呼び出し
- **頻度パラメータ**: `freq_mag`, `freq_baro`, `freq_gps` (単位: サンプル数)
- **ZUPT (Zero Velocity Update)**: 静止判定 → `ESKF.zupt()` を呼び出し

## 検索キーワード（コード内パターン探し用）

```
mex_meukf_step_v2|sensor_updates|update_accel|update_gps|
float64|normalize|struct_to_matlab|get_field_vec3|
estimation_matlab.csv|diff_p|max_error
```

---

## フィードバック・追記事項
本ガイドは 2025/12/21 時点の実装に基づく。以下の変更があれば更新予定:
- PHASE 5: 精度向上後の新しいベンチマーク値
- 新しい MEX ターゲット追加時のビルド例
- よく出るパラメータセット・テストケース
