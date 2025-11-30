# MEUKF vs ESKF — 実装比較と発散/ギザギザ化の原因解析

作成日: 2025-11-30
作成者: 自動生成レポート

## 概要
- 本レポートは、リポジトリ内の `ESKF` 実装（`kalman/ESKF/ESKF.m`）と、同実装に組み込まれた MEUKF ベースの更新 (`update_accel_meukf`, `update_mag_meukf`) の違いを整理し、MEUKF 運用時に観測される「発散しやすい」「ギザギザする」挙動の原因を特定し、対策案を示したものです。

## 参照ファイル
- `kalman/ESKF/ESKF.m` — ESKF 本体（predict / update 実装）
- `kalman/ESKF/Core/meukf_update_attitude.m` — MEUKF (manifold-error UKF) の核となる関数
- その他: `QuaternionLib.*`, `RotationLib.*`, `NoiseEstimator*`, `DivergenceGuard`（ESKF 内で参照）

## 実装上の主な差分（要点）
- 更新手法
  - EKF ブランチ (`update_accel`) は線形化観測行列 H を用いてフル状態に対するカルマンゲインを計算し、ブロック化 / Joseph 形式などで共分散とクロス項を更新している。
  - MEUKF ブランチ (`update_accel_meukf`, `update_mag_meukf`) は姿勢誤差（3×3）サブシステムのみを UKF（誤差空間でのシグマ点）で扱い、得られた姿勢ブロックのみ `obj.P(7:9,7:9)` に代入している（フル状態クロス項は更新していない）。

- 共分散の更新式
  - `meukf_update_attitude.m` の最後で返す共分散は `P_upd = P_sub - K*S*K' + K*R*K'` となっている（+K*R*K' の項が追加されている）。これは通常の形式と整合しない可能性がある。

- UKF パラメータ
  - `meukf_update_attitude.m` のデフォルト `alpha` は `0.1`（実装内で設定）となっており、他の UKF 呼び出し（例: GPS 用）で用いられている `alpha=1e-3` と比べて大きい。sigma-point の分散が相対的に大きくなり得る。

- 安定化/整合化の差
  - EKF 側はクロス項を含めたブロック更新や Joseph 形式、gain clamp、S の正則化を丁寧に行っている。
  - MEUKF 側は姿勢ブロックの対称化や上限クリッピングは行っているが、クロス共分散の整合的更新が欠落している。

## 発散・ギザギザ化の推定原因（優先度付き）
1. クロス共分散（フル状態との相関）を更新していない
   - MEUKF は態勢ブロックだけ置き換えるため、位置・速度・バイアスとの相関が不整合になり、次予測でのイノベーションやゲインが急変してギザギザや発散を生む。

2. 共分散更新式の不整合（+ K*R*K'）
   - `meukf_update_attitude` の P_upd 式は一般的な UKF/EKF の更新と異なり、誤った不確かさ評価を招く可能性がある。

3. UKF パラメータ（alpha）が大きくシグマ点が広がる
   - シグマ点の広がりが大きいと観測関数の非線形性の影響でイノベーションがばらつき、それがギザギザに寄与する。

4. 数値的不安定時のフォールバック挙動（pinv など）
   - S が特異になった際の最終手段（pinv）や強い正則化は一時的に不連続なゲインを生成し得る。

5. 観測次元／正規化の不整合
   - EKF 側は 2 次元（x,y）成分だけを使っている箇所がある一方、MEUKF での h_func の扱い（戻り値の正規化や軸選択）が完全一致していないと差が出る。

## 証拠（コード位置）
- update_accel (EKF): `kalman/ESKF/ESKF.m` 内の `update_accel`（約 line 700 付近）
- update_accel_meukf: `kalman/ESKF/ESKF.m` 内の `update_accel_meukf`（約 line 960 付近）
- meukf_update_attitude: `kalman/ESKF/Core/meukf_update_attitude.m`

## 具体的な修正案（優先度付き）
A. `meukf_update_attitude.m` の安全修正（推奨: まず行う）
   - デフォルト UKF パラメータをリポジトリ内の他の UKF 呼び出しに合わせる: `alpha = 1e-3, beta = 2, kappa = 0`。
   - 共分散の更新式を標準形にする: `P_upd = P_sub - K * S * K'`。（`+ K*R*K'` を削除）
   - 戻り値前に対称化 + 小ジッタで正定値化を保証。

B. MEUKF 更新後にフル状態のクロス共分散を更新する（必須推奨）
   - 流れ: MEUKF で姿勢誤差平均 dtheta と姿勢共分散 P_att_upd を求めた後、観測線形化（または Pxz 近似）を使ってフル状態向けの K_full を計算し、EKF と同じ Joseph 形式／ブロック更新で `obj.P(idx_obs, idx_obs)` と `obj.P(:, idx_obs)` を更新する。
   - 実装方法（概念）:
     1. P_cross = obj.P(:, 7:9) を取得
     2. H_sub を推定（観測モデルのヤコビアン近似、例: 加速度は -skew(g_body) の x,y 成分）
     3. S は MEUKF 内で得られているので K_full = P_cross * H_sub' * inv(S) を計算
     4. Joseph 形式 or ブロック更新でフル P を更新

C. UKF パラメータと R_floor の調整
   - `alpha` を小さくし、`R_floor` を初期では保守的（少し大きめ）にして様子を見る。

D. 数値安定化の統一
   - S の特異時は一貫して「小ジッタ追加 → Cholesky を試行 →（それでも駄目なら）pinv」の順で処理する。pinv は最終手段として扱う。

## 短期の実行プラン（推奨フロー）
1. `meukf_update_attitude.m` のデフォルトパラメータと P_upd 式を安全化（簡単・低リスク）
2. MATLAB で同じ入力データに対して ESKF(EKF) と MEUKF の挙動を比較（`run_simulation.m` を利用）し、イノベーション、P の固有値、カルマンゲインの軌跡をプロットして差を確認する
3. 問題が残る場合、MEUKF 側にフルクロス共分散の更新を実装
4. 最終的に alpha/R_floor をチューニングし、必要ならば more conservative gating（外れ値検出閾値）を導入

## テスト／検証案
- ベースケース: 静止状態（静止→軽微な角変化）で両フィルタを比較。期待: MEUKF は少なくとも EKF と同等か滑らかな応答を示すこと
- 診断出力: 各タイムステップで以下をログ出力
  - イノベーション y (norm)
  - S の条件数 (rcond)
  - カルマンゲイン K の姿勢部分のノルム
  - obj.P の姿勢サブブロック固有値
- 異常時解析: 大きく異なるステップを抽出して、どの更新（MEUKF/EKF）で発生したかを突き合わせる

## 次のアクション候補（選択可能）
- 1) まず `meukf_update_attitude.m` を安全修正して挙動を見る（推奨）
- 2) MEUKF 後にフルクロス共分散を更新するパッチを適用する（やや大きめの変更）
- 3) R と alpha のチューニングを自動化して比較するスクリプトを追加する

---

ファイルパス: `kalman/md/MEUKF_vs_ESKF_analysis.md`

必要であれば、今すぐ (1) の修正（`meukf_update_attitude.m` のデフォルトと P_upd 修正）を適用して比較シミュレーションを実行します。どれを実行しますか？
