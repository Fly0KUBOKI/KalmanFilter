# 修正ファイル一覧 —— 動的センサーdt計算実装

## 修正されたファイル

### MATLAB層
1. **kalman/run_simulation.m**
   - 引数から `dt` 削除（`init`, `step` の呼び出し）
   - 静止時間の計算を時刻累積ベースに変更
   - 各フレームで `sens` 構造体を構築し、時刻情報を追加
   - `prev_time_*` フィールドで前回更新時刻を保持

### C++型定義層

2. **kalman/cpp/Lib/MEUKF/inc/meukf_types.hpp**
   - `SensorData` 構造体：
     - 追加: 時刻フィールド (`current_time`, `prev_time_accel` など)
     - 追加: 個別センサーdt (`dt_accel`, `dt_gyro` など)
     - 削除: 統一的な `dt`

3. **kalman/cpp/Lib/ESKF/inc/interface.hpp**
   - `SensorData` 構造体に時刻情報を追加
   - `Params` 構造体から `dt` を削除

### MEX実装層

4. **kalman/cpp/MEX/mex_hybrid_filter.cpp**
   - `"init"` コマンド：引数チェック `nrhs < 3`（dt削除）
   - `"step"` コマンド：第3引数が `sensor_struct` に変更

5. **kalman/cpp/MEX/Impl/mex_hybrid_filter_impl.hpp**
   - `do_init()`: dt パラメータ削除
   - `do_step()`: センサー構造体から時刻情報を抽出、dt を計算

6. **kalman/cpp/MEX/Impl/mex_hybrid_filter_initializer.hpp**
   - `initialize_eskf_from_matlab()`: dt パラメータ削除

7. **kalman/cpp/MEX/mex_eskf_initializer.cpp**
   - `initialize_eskf_from_matlab()` 実装：
     - 時刻配列から最初のdt を計算
     - 静止サンプル数を時刻累積で計算

## 変更の影響範囲

### 互換性
❌ **破壊的変更**
- MEX インターフェース（`init`, `step` の署名）
- Params 構造体（dt フィールド削除）

### 依存コンポーネント
- MEX層内のセンサー読み込み関数（mex_hybrid_filter_impl.hpp の `do_sensor_update`)
  → 現在は do_step に統合済み

## テスト方法

### 1. コンパイル確認
```bash
cd kalman/cpp/build
build_mex({'mex_hybrid_filter'});
clear mex
```

### 2. 動作確認
```matlab
run_simulation(42, true);
```

### 3. 結果確認
- `Results/estimation_01.csv` を確認
- 数値の妥当性を検証

## 注意事項

1. **フォールバック値**: dt ≤ 0 の場合、10ms を使用
2. **時刻形式**: MATLAB中では `double` 型の秒単位時刻
3. **センサー独立性**: 各センサーは独立して更新可能
4. **初期化**: 最初のフレーム (`k==1`) では prev_time = current_time

## 次のステップ（オプション）

- [ ] センサー周期の多様性をテスト（異なる更新率）
- [ ] GPS とIMU の非同期更新を検証
- [ ] 遅延センサーへの対応
- [ ] イベント駆動更新サポート

