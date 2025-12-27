# ESKF MEX化計画

**作成日**: 2025-12-27  
**更新日**: 2025-12-27  
**状態**: ハイブリッドモードで完了

---

## 📋 結論

完全なMEX化は以下の理由により困難でした：
1. **noiseEstimator**のロジックが複雑（動的ノイズ推定）
2. **R行列**の動的更新が必要
3. センサー更新のパラメータが多く、正確な再現が難しい

### ✅ 採用したアプローチ: ハイブリッドモード

- **predict()**: MEX内部で `mex_adaptive_predict` + `mex_eskf_predict_postprocess` を使用
- **sensor_updates()**: MATLAB ESKF経由で `mex_meukf_step_v2` + `mex_eskf_update_postprocess` を使用
- **zupt()/reset()**: MATLAB実装（内部でMEX関数を使用）

### 📊 バッチテスト結果: 10/10 成功

| メトリック | 平均値 | 最大値 |
|-----------|--------|--------|
| Position RMSE | 0.78 m | 0.82 m |
| Velocity RMSE | 0.58 m/s | 0.59 m/s |
| Roll RMSE | 0.26° | 0.28° |
| Pitch RMSE | 0.28° | 0.30° |
| Yaw RMSE | 0.60° | 0.65° |

---

## 📋 元の構造

```
run_simulation.m
├── ESKF(obs, static_time, dt)     // MATLABクラス初期化
│
├── [ループ内]
│   ├── eskf.predict(a, w)
│   ├── eskf.zupt('check', a, w)
│   ├── eskf.zupt('update')
│   ├── eskf.sensor_updates('accel', a, k)
│   ├── eskf.sensor_updates('mag', m, k)
│   ├── eskf.sensor_updates('baro', p, k)
│   ├── eskf.sensor_updates('gps', lat, lon, alt, k)
│   ├── eskf.reset('check', obs, k)
│   └── eskf.utils('get_euler')
│
└── results.p, results.v, results.euler, ...
```

---

## 🎯 目標の構造

```
run_simulation.m
├── handle = mex_eskf_full('create', obs, static_time, dt)
│
├── [ループ内]
│   └── [p, v, euler, ba, bg] = mex_eskf_full('step', handle, a, w, sensors, k)
│
├── mex_eskf_full('delete', handle)
│
└── results.p, results.v, results.euler, ...
```

**メリット**:
- MATLABオブジェクトのオーバーヘッド削減
- 関数呼び出しの削減（1ステップ1回のMEX呼び出し）
- 完全なC++制御による最適化

---

## 📅 段階的移行計画

### Phase A: MEXインターフェース設計 ✅

**入力/出力の定義**:

#### `mex_eskf_full('create', obs_struct, static_time, dt)`
- **入力**:
  - `obs_struct`: センサーデータ構造体
    - `ax, ay, az`: 加速度 [N×1]
    - `wx, wy, wz`: ジャイロ [N×1]
    - `mx, my, mz`: 磁気 [N×1]
    - `pressure`: 気圧 [N×1]
    - `lat, lon, alt`: GPS [N×1]
    - `time`: 時刻 [N×1]
  - `static_time`: 静止時間 [秒]
  - `dt`: サンプリング時間 [秒]
- **出力**:
  - `handle`: フィルタハンドル [uint64]

#### `mex_eskf_full('step', handle, a, w, sensors, k)`
- **入力**:
  - `handle`: フィルタハンドル
  - `a`: 加速度 [3×1]
  - `w`: ジャイロ（rad/s）[3×1]
  - `sensors`: センサー構造体
    - `mag`: [3×1] (オプション)
    - `pressure`: スカラー (オプション)
    - `lat, lon, alt`: GPS (オプション)
  - `k`: 現在のステップ
- **出力**:
  - `p`: 位置 [3×1]
  - `v`: 速度 [3×1]
  - `euler`: オイラー角（deg）[3×1]
  - `ba`: 加速度バイアス [3×1]
  - `bg`: ジャイロバイアス [3×1]

#### `mex_eskf_full('delete', handle)`
- **入力**: `handle`
- **出力**: なし

---

### Phase B: C++クラス実装

**ファイル**: `kalman/cpp/MEX/mex_eskf_full.cpp`

```cpp
class ESKFFull {
private:
    // 状態
    Vector<3, float> p, v, ba, bg;
    Vector<4, float> q;
    Matrix<15, 15, float> P, Q;
    float dt, g[3];
    
    // パラメータ
    float gps_origin[3];
    float prev_accel[3], prev_mag[3];
    float prev_baro, prev_gps_lat, prev_gps_lon, prev_gps_alt;
    
    // ZUPT
    int zupt_counter;
    bool is_stationary;
    float zupt_threshold_accel, zupt_threshold_gyro;
    int zupt_min_duration;
    
    // フラグ
    int freq_accel, freq_mag, freq_baro, freq_gps;
    int static_samples;
    
public:
    void init(const mxArray* obs, float static_time, float dt);
    void step(const float* a, const float* w, const mxArray* sensors, int k,
              float* p_out, float* v_out, float* euler_out, float* ba_out, float* bg_out);
    void predict(const float* a, const float* w);
    void sensor_updates(const char* type, const float* meas, int k);
    void reset_check(int k);
    void zupt_check(const float* a, const float* w);
    void zupt_update();
};
```

---

### Phase C: 実装ステップ

#### Step 1: 基本構造の作成
- [ ] `mex_eskf_full.cpp`のスケルトン作成
- [ ] ハンドル管理（create/delete）
- [ ] 状態構造体の定義

#### Step 2: 初期化処理
- [ ] `init()`の実装
- [ ] 静止データから初期姿勢計算
- [ ] ノイズパラメータ設定
- [ ] GPS原点設定

#### Step 3: ステップ処理
- [ ] `step()`の実装
- [ ] `predict()`の統合
- [ ] `sensor_updates()`の統合
- [ ] `reset_check()`の統合
- [ ] `zupt_check/update()`の統合

#### Step 4: run_simulation.m修正
- [ ] `run_simulation_mex.m`作成
- [ ] MEX関数呼び出しに変更
- [ ] 結果取得処理

#### Step 5: 検証
- [ ] バッチテスト実行
- [ ] MATLAB版と結果比較

---

## 🔧 実装開始

### Step 1から開始

```matlab
% 使用例（目標）
obs = read_csv('sensor_data.csv');
params = config_params();
dt = mean(diff(obs.time));

handle = mex_eskf_full('create', obs, params.static_time, dt);

for k = 1:length(obs.time)
    a = [obs.ax(k); obs.ay(k); obs.az(k)];
    w = deg2rad([obs.wx(k); obs.wy(k); obs.wz(k)]);
    
    sensors = struct();
    sensors.mag = [obs.mx(k); obs.my(k); obs.mz(k)];
    sensors.pressure = obs.pressure(k);
    if ~isnan(obs.lat(k))
        sensors.lat = obs.lat(k);
        sensors.lon = obs.lon(k);
        sensors.alt = obs.alt(k);
    end
    
    [p, v, euler, ba, bg] = mex_eskf_full('step', handle, a, w, sensors, k);
    
    results.p(:,k) = p;
    results.v(:,k) = v;
    results.euler(:,k) = euler;
end

mex_eskf_full('delete', handle);
```

---

## 📊 予想される改善

| 項目 | 現在 | 目標 |
|------|------|------|
| MEX呼び出し/ステップ | ~10回 | 1回 |
| MATLABオブジェクトオーバーヘッド | あり | なし |
| struct構築オーバーヘッド | 毎ステップ | 最小限 |
| 実行時間 | ~35秒/10000ステップ | ~15秒/10000ステップ (目標) |

---

## 🚨 現在の問題点

### 問題1: Yaw RMSEが約90度
- 初期ヨー角計算または磁気センサー更新に問題がある可能性
- ESKF.mの初期化ロジックとの差異を調査中

### 問題2: Position RMSEがNaN
- 推定位置にNaNが含まれている可能性
- または真値との次元不一致の可能性

### 次のステップ
1. `mex_eskf_full`の初期化と`ESKF.m`の初期化を比較
2. 各センサー更新後の状態を比較
3. `mexCallMATLAB`経由ではなく、既存のMEX関数を直接呼び出す方式に変更を検討

---

## 🚨 注意事項

1. **既存MEX関数の再利用**
   - `mex_adaptive_predict`の内部ロジック再利用
   - `mex_meukf_step_v2`の内部ロジック再利用
   - `mex_sensor_preprocessor`の内部ロジック再利用

2. **数値精度の維持**
   - MATLAB版と同等の結果を維持
   - float/double変換に注意

3. **メモリ管理**
   - ハンドルによる状態管理
   - 適切な解放処理

