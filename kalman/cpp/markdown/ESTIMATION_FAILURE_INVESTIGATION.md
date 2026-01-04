# 推定失敗の原因調査レポート

**作成日**: 2026年1月4日  
**問題**: Lib移行後、全10セットのシミュレーションが失敗（Position RMSE 30-70m、正常時は < 1m）

---

## 📊 問題の概要

### 症状

- **失敗時刻**: 16:34:43
- **成功率**: 0/10 (0%)
- **Position RMSE**: 30-70m（正常時: < 1m）
- **Attitude RMSE**: 1-15度（正常時: < 1度）
- **異常な特徴**:
  - **Gyro bias (final)**: 全て `[0.0000, 0.0000, 0.0000]`（正常時は非ゼロ）
  - **Max Innovation**: 全て `0.0000`（センサー更新が機能していない）
  - **estimation_01.csv**: 全行が `0` のみ（状態推定が全く更新されていない）

### 正常時との比較

**正常時（14:52:49 - 移行前）**:
```
Run 1 Summary: PASS
  Position RMSE: Overall=0.8397 m, X=0.1783 m, Y=0.1390 m, Z=0.8087 m
  Velocity RMSE: 0.5719 m/s
  Roll/Pitch/Yaw RMSE: 0.2676 / 0.2765 / 0.5959 deg
  Gyro bias (final): [-0.2314, 0.0380, 0.0037] deg/s  ← 非ゼロ
  Max Innovation: 0.0000
```

**失敗時（16:34:43 - 移行後）**:
```
Run 1: FAILED
  Position RMSE: Overall=55.1084 m, X=49.1611 m, Y=22.2184 m, Z=11.2453 m
  Velocity RMSE: 2.8885 m/s
  Roll/Pitch/Yaw RMSE: 14.3325 / 14.2636 / 3.8313 deg
  Gyro bias (final): [0.0000, 0.0000, 0.0000] deg/s  ← 異常！
  Max Innovation: 0.0000
```

**estimation_01.csv（失敗時）**:
```csv
time,px,py,pz,vx,vy,vz,roll,pitch,yaw,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z,innov_norm,maha_dist
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
0.0025,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
0.005,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
...（全20003行が0）
```

---

## 🔍 根本原因の特定

### 移行で何が変わったか

**Phase 3 (16:30頃)** で以下の変更を実施：
1. `MEX/Inc/` → `MEX/Impl/` にリネーム
2. MEX実装ファイルのインクルードパスを変更
3. **`MEX/Impl/mex_run_eskf_impl.hpp` を新規作成**

### 重大なバグの発見

**ファイル**: `kalman/cpp/MEX/Impl/mex_run_eskf_impl.hpp`  
**関数**: `do_step` (行33-48)

```cpp
inline void do_step(ESKFState* s, const mxArray* obs, int k) {
    int idx = k - 1;
    double a[3], w[3], m[3];
    getAccel(obs, idx, a);
    getGyro(obs, idx, w);
    getMag(obs, idx, m);
    double deg2rad = M_PI / 180.0;
    w[0] *= deg2rad; w[1] *= deg2rad; w[2] *= deg2rad;
    call_predict(s, a, w);
    zupt_check_and_update(s, a, w);
    call_sensor_update(s, "accel", a, 3, static_cast<double>(k));
    call_sensor_update(s, "mag", m, 3, static_cast<double>(k));
    // baro handling and gps handling omitted for brevity here (same as Inc version)
    //                                ^^^^^^^^^ ← ここが問題！
}
```

**問題点**:
- 🚨 **GPSセンサー更新が実装されていない**
- 🚨 **気圧センサー更新が実装されていない**
- 🚨 **リセットチェック処理が実装されていない**
- コメント「omitted for brevity」= 「簡潔化のため省略」と書かれている

### 正しい実装（MEX/Inc版）

**ファイル**: `kalman/cpp/MEX/Inc/mex_run_eskf_impl.hpp`  
**関数**: `do_step` (行48-114)

```cpp
inline void do_step(ESKFState* s, const mxArray* obs, int k) {
    int idx = k - 1;
    double a[3], w[3], m[3];
    getAccel(obs, idx, a);
    getGyro(obs, idx, w);
    getMag(obs, idx, m);
    
    // Convert gyro to rad/s
    double deg2rad = M_PI / 180.0;
    w[0] *= deg2rad; w[1] *= deg2rad; w[2] *= deg2rad;
    
    // Predict
    call_predict(s, a, w);
    
    // ZUPT check
    zupt_check_and_update(s, a, w);
    
    // Sensor updates
    call_sensor_update(s, "accel", a, 3, static_cast<double>(k));
    call_sensor_update(s, "mag", m, 3, static_cast<double>(k));
    
    // ✅ Baro (single型のみ)
    mxArray* baro_field = mxGetField(obs, 0, "pressure");
    if (baro_field) {
        if (mxGetClassID(baro_field) != mxSINGLE_CLASS) {
            mexErrMsgIdAndTxt("mex_run_eskf:type_error", 
                "Expected single (float) array for field 'pressure', but got %s.", 
                mxGetClassName(baro_field));
        }
        const float* pf = (const float*)mxGetData(baro_field);
        double baro = static_cast<double>(pf[idx]);
        double baro_arr[1] = {baro};
        call_sensor_update(s, "baro", baro_arr, 1, static_cast<double>(k));
    }
    
    // ✅ GPS (double only - type conversion removed)
    mxArray* gps_lat = mxGetField(obs, 0, "lat");
    mxArray* gps_lon = mxGetField(obs, 0, "lon");
    mxArray* gps_alt = mxGetField(obs, 0, "alt");
    if (gps_lat && gps_lon && gps_alt) {
        if (mxGetClassID(gps_lat) != mxDOUBLE_CLASS) {
            mexErrMsgIdAndTxt("mex_run_eskf:type_error", 
                "Expected double array for GPS 'lat', but got %s.", 
                mxGetClassName(gps_lat));
        }
        if (mxGetClassID(gps_lon) != mxDOUBLE_CLASS) {
            mexErrMsgIdAndTxt("mex_run_eskf:type_error", 
                "Expected double array for GPS 'lon', but got %s.", 
                mxGetClassName(gps_lon));
        }
        if (mxGetClassID(gps_alt) != mxDOUBLE_CLASS) {
            mexErrMsgIdAndTxt("mex_run_eskf:type_error", 
                "Expected double array for GPS 'alt', but got %s.", 
                mxGetClassName(gps_alt));
        }
        double lat = mxGetPr(gps_lat)[idx];
        double lon = mxGetPr(gps_lon)[idx];
        double alt = mxGetPr(gps_alt)[idx];
        if (!std::isnan(lat) && !std::isnan(lon)) {
            call_gps_update(s, lat, lon, alt, static_cast<double>(k));
        }
    }
    
    // ✅ Reset check
    check_and_reset(s, k);
}
```

---

## 🧪 検証データ

### MEXファイルのタイムスタンプ

```bash
$ ls -lh --time-style=long-iso mex_run_eskf.mexw64
-rwx------+ 1 takut takut 91K 2026-01-04 16:34 mex_run_eskf.mexw64
```

→ **16:34にビルド済み** = 失敗ログと同時刻 = バグを含んだバージョンがビルドされている

### 使用されているファイルの確認

**ビルド時のインクルード**: `mex_run_eskf.cpp` は以下を参照：
```cpp
#include "Impl/mex_eskf_common.hpp"
#include "Impl/mex_run_eskf_impl.hpp"
#include "Impl/mex_helpers.hpp"
```

→ **`MEX/Impl/mex_run_eskf_impl.hpp` が使用されている**  
→ **`MEX/Inc/mex_run_eskf_impl.hpp`（正常版）は使われていない**

---

## 💡 原因の整理

### なぜこのバグが混入したのか

1. **Phase 3の移行作業中**に、`MEX/Impl/mex_run_eskf_impl.hpp` を新規作成
2. **`do_get_state` と `do_free` の実装が不完全だった**ため、最初に修正
3. その際、**`do_step` も不完全だったが見落とした**
4. コメント「omitted for brevity」を**そのまま残してしまった**（= 実装していないことに気づかなかった）
5. コンパイルは通った（構文エラーなし）
6. **実行時に GPS/Baro 更新がスキップ** → IMUのみで推定 → 大幅なドリフト

### なぜコンパイルが通ったのか

- `call_gps_update`, `check_and_reset` などの関数呼び出しがないだけで、構文エラーではない
- 加速度計・磁気センサー・予測の部分は実装されているため、一見動作しているように見えた
- しかし、**GPS補正がないため位置推定が破綻**

### なぜ estimation_01.csv が全て0なのか

MATLAB側のコード確認：
```matlab
state = mex_run_eskf('get_state', handle);
results.p(:,k) = single(state.p);
results.v(:,k) = single(state.v);
```

**仮説**: `get_state` は実装されており、正しく呼ばれている。しかし：
- 初期化後、`step` が正しく動作していない
- → 状態が更新されない
- → `get_state` で返される状態が初期値のまま（全て0）

**より深刻な可能性**: 初期化も失敗している？

再度確認が必要：`do_init` の実装状況

---

## 📈 影響範囲

### 影響を受けた処理

| 処理 | 状態 | 影響 |
|------|------|------|
| 予測（Predict） | ✅ 実装済み | 正常 |
| ZUPT | ✅ 実装済み | 正常 |
| 加速度計更新 | ✅ 実装済み | 正常 |
| 磁気センサー更新 | ✅ 実装済み | 正常 |
| **気圧計更新** | ❌ **未実装** | **高度推定が機能しない** |
| **GPS更新** | ❌ **未実装** | **位置補正が機能しない** |
| **リセット処理** | ❌ **未実装** | **状態リセットが機能しない** |

### 推定精度への影響

- **位置推定**: GPS補正なし → IMUドリフトのみ → 30-70mの誤差
- **高度推定**: 気圧計なし → Z軸が11m前後で固定
- **ジャイロバイアス**: リセット処理なし → バイアス推定が収束しない → `[0,0,0]` のまま
- **姿勢推定**: GPS/磁気センサーのみでは不十分 → 1-15度の誤差

---

## 🛠️ 修正方針

### 修正内容

**ファイル**: `kalman/cpp/MEX/Impl/mex_run_eskf_impl.hpp`  
**対象関数**: `do_step` (行36-48)

**変更**:
```cpp
// 現在（不完全版）
inline void do_step(ESKFState* s, const mxArray* obs, int k) {
    // ... predict, zupt, accel, mag のみ
    // baro handling and gps handling omitted for brevity here (same as Inc version)
}

// 修正後（完全版）
inline void do_step(ESKFState* s, const mxArray* obs, int k) {
    // ... predict, zupt, accel, mag
    
    // ✅ Baro update を追加
    mxArray* baro_field = mxGetField(obs, 0, "pressure");
    if (baro_field) {
        // ... 完全な実装（Inc版からコピー）
    }
    
    // ✅ GPS update を追加
    mxArray* gps_lat = mxGetField(obs, 0, "lat");
    // ... 完全な実装（Inc版からコピー）
    
    // ✅ Reset check を追加
    check_and_reset(s, k);
}
```

**修正手順**:
1. `MEX/Inc/mex_run_eskf_impl.hpp` の `do_step` 実装を参照
2. Baro処理（48-58行）をコピー
3. GPS処理（60-86行）をコピー
4. Reset処理（88-90行）をコピー
5. `MEX/Impl/mex_run_eskf_impl.hpp` に反映

### 検証手順

1. **修正実施**
   ```bash
   # MEX/Impl/mex_run_eskf_impl.hpp を編集
   ```

2. **MEXリビルド**
   ```matlab
   cd kalman/cpp/build
   clear mex
   build_mex()
   ```

3. **単体テスト**
   ```matlab
   cd kalman
   clear mex
   run_simulation(1, false)
   ```

4. **回帰テスト**
   ```matlab
   run_batch_10sets()
   ```

5. **期待される結果**
   - Position RMSE < 1m
   - Attitude RMSE < 1度
   - Gyro bias が非ゼロ（-0.5 ~ +0.5 deg/s程度）
   - Max Innovation（今後実装予定）

---

## 📝 今後の対策

### 短期対策（即時実施）

1. ✅ **このドキュメントを作成** → 原因を明確化
2. 🔄 **`do_step` のバグ修正** → GPS/Baro/Reset処理を追加
3. 🔄 **MEXリビルド＆テスト** → 正常動作を確認

### 中期対策（次回移行時）

1. **コピー＆ペーストの徹底検証**
   - 新規ファイル作成時は、元ファイルの全機能を網羅することを確認
   - コメント「omitted for brevity」を**絶対に残さない**

2. **段階的な検証**
   - ファイルごとにコンパイル → 単体テスト → 回帰テスト
   - 一度に複数ファイルを変更しない

3. **自動テストの強化**
   - `run_simulation(1, false)` を移行の各段階で実行
   - 出力CSVの最初の数行を目視確認（全て0でないことを確認）

### 長期対策（プロセス改善）

1. **MEX実装のユニットテスト**
   - C++側で個別関数をテスト（GPS更新、Baro更新など）
   - MATLAB側のMEX呼び出し前に検証

2. **コード生成スクリプト**
   - `Inc` → `Impl` 移行を自動化（手動コピーを廃止）
   - テンプレート展開ツールの導入

3. **ドキュメント自動生成**
   - 各MEX関数の入出力仕様を自動抽出
   - 実装漏れをチェックリスト化

---

## 📚 参考資料

- [LIB_MIGRATION_PLAN_FINAL.md](LIB_MIGRATION_PLAN_FINAL.md) - 移行計画書
- [kalman/cpp/MEX/Inc/mex_run_eskf_impl.hpp](../MEX/Inc/mex_run_eskf_impl.hpp) - 正常動作版（参照用）
- [kalman/cpp/MEX/Impl/mex_run_eskf_impl.hpp](../MEX/Impl/mex_run_eskf_impl.hpp) - バグ版（要修正）
- [batch_10sets_log_20260104_163443.txt](../Results/log/batch_10sets_log_20260104_163443.txt) - 失敗ログ
- [batch_10sets_log_20260104_145249.txt](../Results/log/batch_10sets_log_20260104_145249.txt) - 成功ログ（移行前）

---

## ✅ 結論

### 原因

**`MEX/Impl/mex_run_eskf_impl.hpp` の `do_step` 関数が不完全**（GPS・Baro・Reset処理が未実装）

### 影響

- GPS位置補正なし → 位置RMSE 30-70m
- 気圧計補正なし → 高度RMSE 11m
- リセット処理なし → ジャイロバイアス推定が機能しない

### 実施した修正

1. ✅ **`do_step` 関数を完全実装**（GPS・Baro・Reset処理を追加）
2. ✅ **MEXリビルド成功** (16:46:04)
3. ❌ **回帰テスト結果**: **改善せず** (16:47:19)

### 修正後の状況（16:47:19）

**問題が継続**:
- Position RMSE: 依然として 30-70m
- Gyro bias (final): 依然として `[0.0000, 0.0000, 0.0000]`
- ファイル書き込みエラーも発生（Run 3, 5でtruth_data.csvが開けない）

**推定結果の特徴**:
- 初期期間（0-5秒）: 全て0
- 5秒以降: 値が出始めるが、バイアスは常に0

### 追加調査が必要な事項

1. **`do_step`が本当に呼ばれているか？**
   - デバッグ出力を追加して確認
   - GPS更新部分に到達しているか確認

2. **GPS更新関数が正しく動作しているか？**
   - `call_gps_update` の内部実装を確認
   - GPSデータがNaNでないことは確認済み（lat=36, lon=140）

3. **初期化の問題？**
   - `do_init` が正しく状態を初期化しているか
   - 静的期間（0-5秒）の処理が正しいか

4. **MEXバージョンの確認**
   - 本当に新しいバイナリが使われているか
   - `clear mex` が機能しているか

### 次のアクション（緊急）

1. **`MEX/Inc/mex_run_eskf_impl.hpp`（動作確認済み版）を使用**
   - `MEX/mex_run_eskf.cpp` のインクルードを `Inc` に戻す
   - 動作確認後、再度Impl移行を慎重に行う

2. **デバッグ出力追加**
   - `do_step` 内で `mexPrintf` を使ってGPS更新が呼ばれているか確認
   - 初期化時の状態値を出力

3. **比較テスト**
   - Inc版とImpl版で同じseedで実行し、差分を確認

---

**作成者**: GitHub Copilot (Claude Sonnet 4.5)  
**最終更新**: 2026年1月4日 17:00  
**ステータス**: 🚨 **修正未完了 - 追加調査が必要**
