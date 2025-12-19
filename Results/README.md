# Results ディレクトリ - ファイル説明

このディレクトリはシミュレーション実行結果の出力先です。

## ⚠️ ファイル管理ポリシー

**重要**: 以下のファイルは `.gitignore` で除外されており、Git 管理外です。

### 削除対象ファイル（Git 管理外）

| ファイル/パターン | 説明 | 削除理由 |
|-----------------|------|---------|
| `*.csv` | 推定値結果（バッチごと） | 再現可能、容量大 |
| `*.mat` | MAT 形式の記録 | 一時ファイル、再生成可能 |
| `*.txt` | デバッグ出力・トレース | 開発用、不要 |
| `record_*.mat` | センサー更新レコード | 中間ファイル |
| `*trace*` | 状態トレース情報 | デバッグ出力 |
| `*debug*.txt` | デバッグログ | 本番不要 |

### 例

```
kalman/Results/estimation_*.csv
kalman/Results/diff_profile.csv
kalman/Results/state_trace.txt
kalman/Results/dump_records_*.txt
kalman/Results/record_before_*.mat
kalman/Results/*trace*.mat
```

## 推奨運用方法

1. **実行ごとに Results をクリーンアップ**
   ```bash
   rm -f kalman/Results/*.csv kalman/Results/*.mat kalman/Results/*.txt
   ```

2. **解析結果を保存する場合は別フォルダ** 
   - `analysis/run_20251219/` など日付フォルダを作成

3. **分析・比較には以下をコピー**
   - 必要な推定値 CSV
   - ログファイル  
   - README（このファイル）

## Git 管理について

- `.gitignore` で `Results/` 配下の CSV/MAT/TXT は除外済み
- `README.md` のみ管理対象
- ブランチ間での Results ファイルの移行なし

## 関連スクリプト

- `kalman/run_simulation.m` → Results に出力を生成
- `kalman/tools/compare_estimations.m` → 差分分析
