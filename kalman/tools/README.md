# kalman/tools — Phase3 Analysis and Diagnostics

このフォルダには ESKF/Phase3 の差分解析とデバッグに必要なスクリプト群が含まれています。

## 共通ユーティリティ

### `kalman_tools_utils.m` (クラス)
すべてのスクリプト共通で使用する静的メソッド群。パス解決、ファイル I/O、データ抽出ヘルパーを提供。

**主要なメソッド**:
- `get_project_root()` — プロジェクトルート取得
- `get_results_dir()` — Results ディレクトリ（kalman-level または parent-level）
- `read_sensor_data()` — `sensor_data.csv` 読み込み
- `read_truth_data()` — `truth_data.csv` 読み込み
- `parse_top_diffs(max_count)` — `top_diffs.txt` をパース
- `pick_main_var(S, vars)` — 構造体から主要変数抽出
- `safe_get(s, name)` — ネストされた構造体から安全に取得
- `find_nearest_samples()` — 時刻から最近傍サンプル取得
- `compute_rmse()`, `matrix_delta_fro()` — 数値計算ヘルパー

## 常用スクリプト

### `dump_records_for_top_diffs.m`
`top_diffs.txt` に列挙された時刻に対応する `record_before_*` / `record_after_*` ファイルをテキスト形式でダンプ。

```matlab
dump_records_for_top_diffs()
```

出力: `Results/dump_records_time_*.txt`

---

### `generate_record_pairs_for_top_diffs.m`
`top_diffs.txt` の時刻に対応するサンプル索引を決定し、ESKF を実行して各センサー更新前後の状態を記録。

```matlab
generate_record_pairs_for_top_diffs()          % 最初の50個
generate_record_pairs_for_top_diffs(20)        % 最初の20個
```

出力: `Results/record_before_*.mat`, `Results/record_after_*.mat`

---

### `extract_record_before_after_P.m`
`record_before/after` ペアから共分散行列 `P`、ゲイン `K`、イノベーション `y` を抽出し集約。

```matlab
extract_record_before_after_P()
extract_record_before_after_P(out_mat, out_txt)
```

出力: `Results/extracted_record_pairs.mat`, `Results/extracted_record_pairs.txt`

---

### `compare_estimations.m`
2つの推定結果 CSV ファイルの RMSE を比較。真値と対比して差分を評価。

```matlab
compare_estimations('Results/estimation_matlab.csv', 'Results/estimation_mex.csv')
```

出力: コンソール表示（RMSE 比較）

---

### `compare_estimations_mex.m`
MEX 版推定結果（複数の `estimation_mex_*.csv`）を集計・比較。バッチ実行結果の統計。

```matlab
compare_estimations_mex()
```

---

### `dump_mat_fields.m`
MAT ファイルの構造を視覚的に表示（デバッグ用）。

```matlab
dump_mat_fields('Results/record_before_accel_2001.mat')
```

---

## 作業フロー例

### Phase3 の差分原因調査

1. **top-diffs を特定** — 推定誤差が大きい時刻を `top_diffs.txt` に出力
2. **レコードペア生成** — `generate_record_pairs_for_top_diffs(20)`
3. **共分散抽出** — `extract_record_before_after_P()`
4. **ダンプ表示** — `dump_records_for_top_diffs()`
5. **比較** — `compare_estimations('matlab.csv', 'mex.csv')`

### MATLAB vs MEX の RMSE 比較

```matlab
compare_estimations('Results/estimation_matlab_from_kalman01.csv', 'Results/estimation_mex.csv')
```

---

## ファイル一覧

| ファイル | 用途 | 再生成 |
|---------|------|--------|
| `kalman_tools_utils.m` | 共通ユーティリティ（クラス） | ✗ |
| `dump_records_for_top_diffs.m` | ダンプ実行 | ✓ |
| `generate_record_pairs_for_top_diffs.m` | レコード生成 | ✓ |
| `extract_record_before_after_P.m` | P/K/y 抽出 | ✓ |
| `compare_estimations.m` | 2ファイル RMSE 比較 | ✓ |
| `compare_estimations_mex.m` | バッチ集計 | ✓ |
| `dump_mat_fields.m` | MAT 構造表示（デバッグ） | ✓ |

---

## 注記

- すべてのスクリプトは `kalman_tools_utils` に依存。変更しないこと。
- `Results/` ディレクトリは自動作成される（存在しない場合）。
- 大量レコード生成時は時間がかかる（`generate_record_pairs_for_top_diffs(50)` は数分）。
