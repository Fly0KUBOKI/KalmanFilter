# MATLAB KalmanFilter 環境依存性の問題診断ガイド

## 概要

コンパイルされたMEXバイナリ（`.mexw64`）を実行しているにもかかわらず、異なる環境で結果が変わるのは、以下の6つの主要な原因が考えられます。

---

## 【重要】根本的な原因カテゴリ

### 1. **MATLAB環境の不一致（最も重要）**
MEXバイナリは、ビルド時のMATLABバージョン・アーキテクチャに依存します。

#### チェック項目
```matlab
% コマンドラインで実行
ver MATLAB          % MATLABバージョン確認
computer            % アーキテクチャ確認 (PCWIN64, GLNXA64など)
mexext              % MEX拡張子確認 (.mexw64 = Windows 64-bit)
```

**問題パターン:**
- ビルド環境: MATLAB R2021a, Windows 64-bit → `.mexw64` 生成
- 実行環境1: MATLAB R2021a, Windows 64-bit → ✅ 動作
- 実行環境2: MATLAB R2024a, Windows 64-bit → ❌ 数値結果変更（メモリレイアウト変更の可能性）

**解決方法:**
```
各環境で MEX を再ビルドする必要があります。
kalman/cpp/build/build_mex.m を実行して環境専用バイナリを生成してください。
```

---

### 2. **初期化パラメータの環境依存（次に重要）**
初期化パラメータ（GPS原点、ノイズパラメータ、静止時間など）が環境ごとに異なる場合、結果が大きく変わります。

#### チェック項目
```matlab
% config_params.m の以下の値を確認
params.gps_origin.lat = 36.0;       % GPS原点 纬度
params.gps_origin.lon = 140.0;      % GPS原点 経度
params.static_time = 5;             % 初期化期間（秒）
params.dt = 0.0025;                 % サンプリング時間（秒）

% ノイズパラメータ
params.noise.accel_std  = 0.1;      % 加速度ノイズ
params.noise.gyro_std   = 0.5;      % ジャイロノイズ
params.noise.mag_std    = 5.0;      % 磁気ノイズ
```

**問題パターン:**
- 環境1: `static_time = 5秒` で初期化 → 正常に動作
- 環境2: `static_time = 2秒` で初期化 → 初期Roll/Pitch推定が不正確 → 以降全て失敗

**解決方法:**
```matlab
% 全環境で同じパラメータを使用するために、
% config_params.m をプロジェクトルートに置いて、
% すべてのスクリプトから参照するようにしてください
```

---

### 3. **センサーデータの形式・範囲の違い（高重要度）**
MEXは入力データの形式に非常に厳密です。型混在や配列形状の誤りがあると、数値結果が大きく変わります。

#### チェック項目

**A. データ型の一致確認**
```matlab
% run_simulation.m でセンサーデータを読み込んだ直後に以下を実行
obs = read_observation(proj_root);

% 各要素の型を確認
fprintf('ax class: %s\n', class(obs.ax));        % → 'single' であること
fprintf('wx class: %s\n', class(obs.wx));        % → 'single' であること
fprintf('mx class: %s\n', class(obs.mx));        % → 'single' であること
fprintf('lat class: %s\n', class(obs.lat));      % → 'double' であること ★GPS のみ double

% 配列形状を確認
fprintf('ax size: %s\n', mat2str(size(obs.ax))); % → [1, n] または [n, 1] のいずれか
fprintf('lat size: %s\n', mat2str(size(obs.lat)));
```

**B. データ値の範囲確認**
```matlab
% センサー値が物理的に合理的な範囲内か確認
fprintf('加速度 [ax min, max]: [%.3f, %.3f] m/s²\n', min(obs.ax), max(obs.ax));  % ≈ ±1.0 が標準
fprintf('ジャイロ [wx min, max]: [%.3f, %.3f] deg/s\n', min(obs.wx), max(obs.wx)); % ≈ ±10 が標準
fprintf('GPS [lat min, max]: [%.6f, %.6f] rad\n', min(obs.lat), max(obs.lat));
fprintf('GPS [lon min, max]: [%.6f, %.6f] rad\n', min(obs.lon), max(obs.lon));
fprintf('GPS [alt min, max]: [%.2f, %.2f] m\n', min(obs.alt), max(obs.alt));
```

**問題パターン:**
- 環境1: GPS入力が `double` 型で radian 単位 → ✅ 正常
- 環境2: GPS入力が `single` 型で degree 単位 → ❌ 座標系混在 → 推定失敗

**解決方法:**
```
read_csv() や load_sim_data() で型変換ロジックを確認してください。
すべての環境で以下を満たすようにしてください：
  - ax, ay, az, wx, wy, wz, mx, my, mz, pressure → single
  - lat, lon, alt → double
```

---

### 4. **MEXバイナリキャッシュの残存**
MATLAB はMEXバイナリをメモリにキャッシュします。古いバイナリが実行される場合があります。

#### チェック項目
```matlab
% 古いMEXキャッシュをクリア
clear functions     % すべての関数をアンロード
clear mex          % すべてのMEXをアンロード
rehash             % パスキャッシュを再構築

% その後、run_simulation を実行
run_simulation(42, false);
```

**問題パターン:**
- 開発環境で MEX をビルド更新したが、古いバイナリがメモリに残っている
- 結果に予期しない変動が発生する

**解決方法:**
```matlab
% 毎回実行前に以下を追加
clear functions; clear mex; rehash;
```

---

### 5. **浮動小数点演算の環境依存（中重要度）**
CPUキャッシュやメモリアラインメントの違いにより、`float` の丸めモードが微妙に変わります。

#### チェック項目
```cpp
// C++ コード内の以下を確認 (mex_run_eskf.cpp など)

// NG例: double と float の混在キャスト
double lat = obs_double[0];                 // GPS入力
float lat_f = (float)lat;                   // ここで精度喪失
float pos = kalman_compute(lat_f, ...);     // 精度喪失した値で計算

// OK例: 最小限のキャスト
float lat_f = (float)obs_double[0];         // 一度だけキャスト
float pos = kalman_compute(lat_f, ...);     // 一度のみ変換
```

**問題パターン:**
- 環境1: CPUが AVX2 対応 → 演算最適化で丸めが異なる
- 環境2: CPUが AVX2 非対応 → 丸めが異なる

**解決方法:**
```
この問題は軽微です。推定結果の差が mm 単位なら環境依存の演算誤差です。
重大な失敗（数メートル以上の誤差）であれば、上記 1-4 を確認してください。
```

---

### 6. **ランダム数列の初期化（低重要度だが重要な場合がある）**
`sim_generate()` でセンサーノイズを生成する際、乱数シードが異なるとノイズパターンが変わります。

#### チェック項目
```matlab
% run_simulation() の呼び出しを確認
run_simulation(seed, skip_data_gen);  % seed は何か？

% seed が明示的に指定されているか確認
% seed = [] の場合、ランダムシードが毎回異なります
rng(seed, 'twister');  % このコマンドが実行されているか？
```

**問題パターン:**
- 環境1: `run_simulation(42, false)` で seed=42 固定 → 同じノイズが生成される
- 環境2: `run_simulation([], false)` で seed 未指定 → 異なるノイズが生成 → 推定結果が異なる

**解決方法:**
```matlab
% すべての環境で同じシードを使用
run_simulation(42, false);  % seed を固定
% または
run_batch_10sets();  % batch は 1-10 の seed で統計を取る
```

---

## 【実行手順】環境依存性の診断・解決ステップ

### ステップ 1: 両環境でMATLAB情報を確認
```matlab
% 環境1 と環境2 の両方で実行して比較

fprintf('=== MATLAB Environment Info ===\n');
disp(ver('MATLAB'));
fprintf('Arch: %s\n', computer);
fprintf('MEX Extension: %s\n', mexext);
fprintf('Compiler: %s\n', mex.getCompilersAndSDKs);
```

**結果の解釈:**
- MATLABバージョン違い → 環境専用のMEXビルドが必要
- アーキテクチャ違い（32-bit vs 64-bit） → MEXビルド必須

---

### ステップ 2: パラメータ一致確認
```matlab
% 両環境で実行して結果を比較
proj_root = fileparts(mfilename('fullpath'));
params = config_params();

fprintf('=== Parameters ===\n');
fprintf('static_time: %.1f\n', params.static_time);
fprintf('dt: %.4f\n', params.dt);
fprintf('GPS origin: [%.6f, %.6f, %.2f]\n', params.gps_origin.lat, params.gps_origin.lon, params.gps_origin.alt);
fprintf('accel_std: %.3f\n', params.noise.accel_std);
fprintf('gyro_std: %.3f\n', params.noise.gyro_std);
```

**結果の解釈:**
- 値が異なる → `config_params.m` を統一する
- 値が同じ → ステップ3へ

---

### ステップ 3: センサーデータ型確認（重要）
```matlab
% 両環境で実行
clear functions; clear mex; rehash;
obs = read_observation(proj_root);

fprintf('=== Sensor Data Types ===\n');
fprintf('ax: %s [%.6f, %.6f]\n', class(obs.ax), min(obs.ax), max(obs.ax));
fprintf('wx: %s [%.6f, %.6f]\n', class(obs.wx), min(obs.wx), max(obs.wx));
fprintf('lat: %s [%.10f, %.10f]\n', class(obs.lat), min(obs.lat), max(obs.lat));
fprintf('lon: %s [%.10f, %.10f]\n', class(obs.lon), min(obs.lon), max(obs.lon));
fprintf('alt: %s [%.2f, %.2f]\n', class(obs.alt), min(obs.alt), max(obs.alt));
```

**結果の解釈:**
- ax/wx/mx が `single` でない → `read_csv()` で型変換ロジックを確認
- lat/lon が `double` でない → GPS専用変換ロジックを確認
- 値の範囲が大きく異なる → データ生成ロジックの誤り

---

### ステップ 4: 単一実行で初期化をトレース
```matlab
% 両環境で同じシードで実行
clear functions; clear mex; rehash;
run_simulation(42, false);

% 出力: kalman/Results/estimation_01.csv
% CSVの最初の10行を比較
T = readtable('kalman/Results/estimation_01.csv');
disp(head(T, 10));

% 特に以下の列を確認
% - p_east, p_north, p_up: 位置推定値
% - roll, pitch, yaw: 姿勢推定値
% - ba_x, ba_y, ba_z: 加速度バイアス
```

**結果の解釈:**
- 最初の5-10行が同じ → 初期化は正常
- 最初から異なる → GPS原点またはノイズパラメータが異なる
- 10行目あたりから乖離 → センサーデータ型の不一致

---

### ステップ 5: MEXキャッシュをクリアして再実行
```matlab
% MEXキャッシュがあるなら、明示的にクリア
clear functions; clear mex; rehash;

% 環境1と環境2で同じシードで実行
run_simulation(42, false);

% 結果ファイルを比較
% 結果が同じ → キャッシュが原因
% 結果が異なる → 環境設定の違い
```

---

### ステップ 6: MEXビルド確認
```matlab
% 各環境でMEXを再ビルド
cd kalman/cpp/build
build_mex({'mex_run_eskf'});

% ビルドログを確認
type build_mex_log_*.txt  % 最新のログを表示

% キーワード検出: Warning, Error
% コンパイラオプションが異なる場合、警告が異なります
```

**結果の解釈:**
- Compiler Warning あり → `build_mex.m` の `-O` オプションを確認
- Error あり → C++ソースコードが環境依存

---

## 【迅速な対応フロー】

### パターンA: 新しい環境で初めて実行する場合
```matlab
% 1. MEXを環境専用にビルド
cd kalman/cpp/build
build_mex();         % すべてのMEXを再ビルド
clear mex; rehash;

% 2. パラメータを確認
params = config_params();
fprintf('Config OK: static_time=%.1f, dt=%.4f\n', params.static_time, params.dt);

% 3. 単一実行テスト
clear functions; clear mex; rehash;
run_simulation(42, false);

% 4. 結果確認
T = readtable('kalman/Results/estimation_01.csv');
disp(head(T, 10));
```

### パターンB: 環境1 と環境2 で結果が異なる場合
```matlab
% 【環境1で実行】
run_simulation(42, false);
copyfile('kalman/Results/estimation_01.csv', 'env1_estimation.csv');

% 【環境2で実行】
clear functions; clear mex; rehash;
run_simulation(42, false);
copyfile('kalman/Results/estimation_01.csv', 'env2_estimation.csv');

% 【環境1または2で比較】
T1 = readtable('env1_estimation.csv');
T2 = readtable('env2_estimation.csv');
diff_p = sqrt((T1.p_east - T2.p_east).^2 + (T1.p_north - T2.p_north).^2 + (T1.p_up - T2.p_up).^2);
fprintf('Max position error: %.6f m\n', max(diff_p));

% エラーが大きい → 上記ステップ 1-6 に従う
% エラーが小さい（mm単位） → 環境依存の浮動小数点演算誤差（無視可）
```

---

## 【よくあるエラーメッセージと対処法】

### エラー1: "Invalid MEX-file" または "不正なMEXファイル"
```
原因: MEXバイナリが実行環境と不一致
対処:
  cd kalman/cpp/build
  build_mex();        % 環境専用ビルド
  clear mex;
```

### エラー2: 初期化時に "GPS origin mismatch" など
```
原因: config_params.m が異なる
対処:
  - 両環境で config_params.m の内容を同期
  - git version control で管理する
```

### エラー3: 数値結果が数メートル以上ずれている
```
原因: 最初の5つのいずれか
対処:
  ステップ 1-6 に従う優先順位:
  1. MEXバイナリの環境適合性 → build_mex()
  2. config_params.m の一致 → ファイル同期
  3. センサーデータ型 → read_csv() 確認
  4. MEXキャッシュ → clear mex
  5. 初期化パラメータ → static_time, dt 確認
```

### エラー4: 数mm単位の数値差（小さい）
```
原因: 環境依存の浮動小数点演算誤差
対処: 無視してOK（許容範囲）
```

---

## 【最も可能性の高い3つの原因】

### 1位: MEXバイナリの環境不一致（確率 70%）
**症状:** 新しい環境でいきなり実行したら失敗した  
**確認:** `computer` の出力が異なる、もしくはMATLABバージョン違い  
**解決:** `build_mex()` で環境専用ビルド

### 2位: config_params.m の異なり（確率 20%）
**症状:** static_time や GPS origin が異なる  
**確認:** `fprintf('static_time: %.1f\n', params.static_time);`  
**解決:** ファイル同期、git で管理

### 3位: センサーデータ型の混在（確率 10%）
**症状:** 初期化から数サンプル後に大きくズレる  
**確認:** `class(obs.ax)` が `single` か、`class(obs.lat)` が `double` か  
**解決:** `read_csv()` の型変換ロジック統一

---

## 【まとめ】環境依存性の検査チェックリスト

- [ ] MATLABバージョンが両環境で同じか確認
- [ ] `computer` で 32-bit / 64-bit が一致しているか確認
- [ ] `build_mex()` で各環境専用ビルルルを実行
- [ ] `config_params.m` の値が両環境で同じか確認
- [ ] センサーデータ型（single/double）が正しいか確認
- [ ] GPS値が radian 単位か degree 単位か確認
- [ ] `clear functions; clear mex; rehash;` でキャッシュクリア
- [ ] 同じシード (`run_simulation(42, false)`) で比較実行
- [ ] CSVの最初の5-10行で一致確認

---

