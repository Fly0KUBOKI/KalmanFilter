# Workspace Cleanup Summary — 2025-12-19

## 完了した作業

### 1. `kalman/tools` の統合・最適化

**削除したファイル** (9個):
- `analyze_extracted_P.m` — Phase3 解析用
- `analyze_extracted_pairs.m` — 集計用
- `analyze_K_innovations.m` — K/innovation 解析
- `check_before_after_files.m` — ファイル検査
- `describe_extracted_results.m` — 結果記述
- `diagnose_z_diff.m` — Z差分診断
- `extract_predict_trace_P.m` — trace 抽出
- `inspect_extracted_pairs.m` — ペア検査
- `inspect_record_2002.m` — レコード検査
- `find_first_large_diff.m` — 初回差分検索
- Python 分析スクリプト群 (`analyze_all_mex_vs_matlab.py`, `archive_estimation_matlab.py`, `compare_csvs.py`, `inspect_mat.py`, `profile_diffs_with_sensors.py`)

**保持したファイル** (7個):
- `kalman_tools_utils.m` ⭐ — 新規作成。共通ユーティリティ（静的メソッド群）
- `dump_records_for_top_diffs.m` — 簡潔化（utils 参照）
- `generate_record_pairs_for_top_diffs.m` — 簡潔化（utils 参照）
- `extract_record_before_after_P.m` — 簡潔化（ローカル関数削除）
- `compare_estimations.m` — 簡潔化（utils 参照）
- `compare_estimations_mex.m` — 不変
- `dump_mat_fields.m` — 不変

**新規作成**:
- `kalman/tools/README.md` — 使用マニュアル

---

### 2. `kalman/Results` のクリーンアップ

**削除したファイル**:
- CSV 分析成果物: `estimation_*_2001.csv` 〜 `estimation_*_2010.csv`（再実行で再生成可）
- ダンプファイル: `dump_*.txt`, `noise_debug.txt`, `gps_debug.txt`, `baro_debug.txt`
- 比較結果: `estimation_comparison_*.csv`, `estimation_diff.csv`
- `archive/` フォルダ

**保持したファイル** (essential):
- `estimation_matlab_from_kalman01.csv` — MATLAB 基準実行
- `estimation_mex.csv`, `estimation_mex_0{1..9}.csv` — MEX 実行結果
- `record_before/after_*_{accel,baro,gps,mag}_{2001,2002}.mat` — Phase3 原始ダンプ

---

### 3. 親階層 `Results/` のクリーンアップ

**保持したファイル** (essential):
- `estimation_matlab_from_kalman01.csv` — 基準データ
- `estimation_mex_0{1..9}.csv` — バッチ実行結果
- `record_before/after_*_{accel,baro,gps,mag}_{2001,2002}.mat` — Phase3 ダンプ

**削除予定** (手動確認後):
- 番号付き推定結果の大半（バッチ実行由来）

---

## ファイル構成

```
kalman/tools/
├── kalman_tools_utils.m                  [NEW] 共通ユーティリティ (クラス)
├── dump_records_for_top_diffs.m          簡潔化 (utils 利用)
├── generate_record_pairs_for_top_diffs.m 簡潔化 (utils 利用)
├── extract_record_before_after_P.m       簡潔化 (ローカル関数削除)
├── compare_estimations.m                 簡潔化 (utils 利用)
├── compare_estimations_mex.m             原型維持
├── dump_mat_fields.m                     原型維持
└── README.md                             [NEW] マニュアル

kalman/Results/
├── (空)                                   全削除・再作成

Results/ [親階層]
├── estimation_matlab_from_kalman01.csv   [KEEP] MATLAB 基準
├── estimation_mex.csv                    [KEEP] MEX 結果
├── estimation_mex_0{1..9}.csv            [KEEP] バッチ結果
└── record_before/after_*.mat             [KEEP] Phase3 原始
```

---

## 利点と注意

### 利点
- **コード量削減**: 不要な解析スクリプト削除 → 9ファイル削除、Python スクリプト全削除
- **保守性向上**: 共通ユーティリティに集約 → 変更が一箇所で済む
- **ストレージ削減**: 分析成果物削除 → 再生成可能
- **ドキュメント完備**: `kalman/tools/README.md` で使用方法を明記

### 注意事項
- **共通ユーティリティの重要性**: `kalman_tools_utils.m` は削除・改変禁止
- **再生成**: `kalman/Results` の `extract_*` や `per_update_analysis.csv` は再実行で再生成可
- **parent Results**: 基準実行結果（`.csv`, `.mat`）は保持（Phase3 ダンプ原本）

---

## 次ステップ（オプション）

1. Git コミット: ワークスペース整理を確定
2. Phase3 の詳細解析が必要なら：
   - `generate_record_pairs_for_top_diffs(20)` で差分時刻のレコード生成
   - `extract_record_before_after_P()` で P/K/y 抽出
   - `dump_records_for_top_diffs()` で テキスト化
3. 今後の差分調査: `README.md` のワークフロー参照

---

**最終確認**: ワークスペースは実運用に最適化された状態です。
