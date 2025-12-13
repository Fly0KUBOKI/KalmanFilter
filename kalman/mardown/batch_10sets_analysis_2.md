# 10セットバッチテスト 再解析レポート

**実行日時**: 2025年12月7日 11:57 - 12:03（約6分）

---

## 📌 概要
- 実行: `run_batch_10sets.m` による10回バッチ
- 結果保存先: `kalman/Results`
- 総合結果: **成功 10/10 (100%)**

---

## 🔢 統計（成功した10Runの集計、ログ出力値に基づく）
- Position RMSE: Mean = 1.8747 m, Std = 0.7664, Max = 2.9042 m
- Velocity RMSE: Mean = 0.5659 m/s, Std = 0.0829, Max = 0.7885 m/s
- Roll RMSE: Mean = 0.8348 deg, Std = 0.4758, Max = 1.8502 deg
- Pitch RMSE: Mean = 1.2695 deg, Std = 0.9908, Max = 3.4731 deg
- Yaw RMSE: Mean = 2.2328 deg, Std = 0.3424, Max = 2.7606 deg

---

## 個別Runの注目点（CSV 値より抜粋）
- Run 1: Pos=2.3746 m, Att=0.52/0.30/1.71 deg, MaxInnov=0.9072
- Run 2: Pos=1.3391 m, Att=1.10/1.47/2.20 deg, MaxInnov=1.5369
- Run 3: Pos=2.6075 m, Att=0.64/1.10/2.12 deg, MaxInnov=1.0192
- Run 4: Pos=2.9042 m (最大), Att=1.85/3.47/2.18 deg, MaxInnov=2.6689 (最大イノベーション)
- Run 5: Pos=2.3443 m, Att=1.29/1.36/2.76 deg, MaxInnov=1.6268
- Run 6: Pos=1.1112 m, Att=0.69/0.31/2.06 deg, MaxInnov=1.1394
- Run 7: Pos=1.0242 m（最小）, Att=0.59/1.21/2.58 deg, MaxInnov=1.3506
- Run 8: Pos=1.2902 m, Att=0.42/0.53/2.16 deg, MaxInnov=0.9451
- Run 9: Pos=2.6770 m, Att=0.98/2.31/2.69 deg, MaxInnov=1.5414
- Run10: Pos=1.0744 m, Att=0.27/0.64/1.87 deg, MaxInnov=0.8832

※ Max Innovationは `batch_10sets_summary.csv` の `Max_Innov` に基づく

---

## 発散・健全性チェック
- NaN/Inf: 10Runとも検出なし（`Has_NaN=0`, `Has_Inf=0`）
- 最大イノベーション: 2.6689（Run 4） → 現在の閾値 5.0 に対して十分な余裕あり
- Position RMSE の閾値チェック: 全Runで 10.0 m より小さいため合格判定

---

## 考察
1. 安定性
   - 変化量クラッピングとイノベーション閾値の緩和（5.0）は有効。発散は発生せず、NaN/Infも無い。成功率100%は堅牢性向上を示す。

2. 精度バラつき
   - 位置RMSEは Run 間で 1.02～2.90 m とやや幅がある（Std=0.77m）。これはノイズ実現ごとの差による自然なばらつきで、全体として許容範囲内。
   - Pitch と Yaw のばらつきが Roll より大きい。 yaw は地磁気に依存するため、地磁気ノイズ/初期条件で差が出やすい。

3. イノベーション
   - 最大値は Run 4 の 2.6689。MEUKF の性質上、イノベーション分布はやや広がることがあるが、今回の値は安全域に収まる。

---

## 推奨アクション
- 現行設定を本番候補として維持してよい（安定性・精度とも良好）。
- 追加テスト案:
  - 長時間試験 (例: 1000秒以上) でバイアス推定の収束挙動を確認
  - 環境依存性評価（地磁気摂動、加速度ピーク入力）
  - クラッピング倍率の微調整（現在 3.0 → 2.5/3.5）と精度影響の確認
- パフォーマンス改善: MEX 化により predict/update の呼び出しコストを削減可能

---

## ファイル
- サマリーCSV: `kalman/Results/batch_10sets_summary.csv`
- 実行ログ: `kalman/Results/batch_10sets_log.txt`
- 個別推定: `kalman/Results/estimation_01.csv` ～ `estimation_10.csv`
- 保存レポート（本ファイル）: `kalman/md/batch_10sets_analysis_2.md`

---

作成日: 2025-12-07
作成者: GitHub Copilot
