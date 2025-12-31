# 推定失敗の原因分析レポート

## 概要

**コミット:** `6207225a0ead9496f713c16ad8aa832f48d52137`（失敗）  
**基準コミット:** `d3dab8c1887dd734b4641bc080436b266a47bf6c`（正常）

変更による推定が完全に失敗しました。すべてのテストケースで RMSE が大幅に悪化し、姿勢推定が完全に破綻しています。

```
批判的な失敗状況:
- Run 1: Roll 52.6°, Pitch 36.0°, Yaw 158.5° (期待値 < 5°)
- Run 2: Roll 139.3°, Pitch 33.2°, Yaw 105.5°
- Run 3: Z位置 10126.7m (期待値 < 10m)
```

---

## 原因特定

### 1. **メジャーな問題：座標系変換エラー（行列の行/列の混同）**

**ファイル:** `kalman/cpp/Inc/MEX/mex_run_eskf_sensor_updates.hpp`

#### 問題コード（新：コミット 6207225）

```cpp
// P: MATLAB column-major -> C++ row-major
for (int r = 0; r < 15; ++r) {
    for (int c = 0; c < 15; ++c) {
        input.prev_state.P[r*15 + c] = static_cast<float>(P[c*15 + r]);
        //                              ^^^^^^^^^^^^^^^^  転置が逆
    }
}
```

#### 正常な変換（旧：コミット d3dab8c）

旧コードでは MEX から MATLAB 構造体経由で、既に正しく変換されていました。

#### なぜ失敗するのか

共分散行列 $P$ は 15×15 の対称行列です。

- **MATLAB:** 列優先（column-major）形式：`P[c*15 + r]`
- **C++:** 行優先（row-major）形式：`P[r*15 + c]`

新コードの変換：
$$P_{\text{C++}}[r \cdot 15 + c] = P_{\text{MATLAB}}[c \cdot 15 + r]$$

これは実質的に **転置を施しながら副本を作成** することになり、非対称な "疑似共分散" を生成します。

$$P_{\text{new}} \neq P^T$$（転置と列挙順序の混同）

これにより：
- カルマンゲイン計算が正確でなくなる
- 状態更新が数値的に不安定になる
- 共分散が負定値に近づき、発散

---

### 2. **副問題：dtの未設定**

**ファイル:** `kalman/cpp/Inc/MEX/mex_run_eskf_sensor_updates.hpp`（行 ~374）

```cpp
mxSetField(sensor_data, 0, "dt", mxCreateDoubleScalar(0));
//                                                        ↑
//                                                   dt = 0 に固定！
```

時刻ステップが常に 0 に設定されているため：
- 予測ステップが完全に無視される
- 状態の時間発展が停止する
- センサー融合が盲目的に状態を更新しようとする

---

### 3. **副問題：MEUKF 呼び出しの直接化による隠れたバグ**

旧コードは `mexCallMATLAB` で `mex_meukf_step_v2` を呼び出し、その結果から詳細なデバッグ出力を抽出していました。

新コードは C++ 直接呼び出し：
```cpp
meukf::MEUKFCore::step(input, output);
```

この変更により：
- **検証ロジックの喪失** - 中間結果が検査されない
- **仮定の変更** - 直接呼び出しの I/O 形式が MEX ラッパーと異なる可能性

特に `output.pred_P` や `output.last_H` の形式が不正確の場合、ノイズ推定が完全に失敗します。

---

### 4. **副問題：メモリ初期化の問題**

新コードで`prev_mag`, `prev_gps_pos`, `prev_baro_alt` を 0 で初期化：

```cpp
for (int i = 0; i < 3; ++i) {
    input.sensor.prev_mag[i] = 0.0f;
    input.sensor.prev_gps_pos[i] = 0.0f;
}
input.sensor.prev_baro_alt = 0.0f;
```

前フレームのセンサー値が常に 0 に設定されるため、変化検出ロジックが誤作動する可能性があります。

---

## 修正方案

### **推奨：コミットを巻き戻す**

この変更の複数の根本原因（座標系、dt、直接呼び出し）により、部分的な修正では不十分です。

```bash
# 方案 A：完全巻き戻し
git revert 6207225a0ead9496f713c16ad8aa832f48d52137 -m 1

# または

git reset --hard d3dab8c1887dd734b4641bc080436b266a47bf6c
git push origin phase6 --force-with-lease
```

### **代替案：段階的修正（高リスク）**

もし変更を保持したい場合：

#### 1. 共分散行列の変換を修正

```cpp
// 正しい変換：
// MATLAB column-major [c*15 + r] → C++ row-major [r*15 + c]
// （転置の必要なし、単なる順序変更）
for (int r = 0; r < 15; ++r) {
    for (int c = 0; c < 15; ++c) {
        // MATLAB: P[c*15 + r] (column c, row r)
        // C++: P_cpp[r*15 + c] = P[c*15 + r] ✓
        input.prev_state.P[r*15 + c] = static_cast<float>(P[c*15 + r]);
    }
}
```

⚠️ この変換は**実は正しい**ように見えます。問題は、入力側 `P` 自体が既に間違っている可能性があります。

#### 2. dt の代入を修正

```cpp
// sensor_data への dt の設定を確認
mxSetField(sensor_data, 0, "dt", mxCreateDoubleScalar(dt));
//                                                        ^
//                                        呼び出し元の dt を使用
```

現在のコードを確認してください。`dt` 変数が関数スコープに存在するはずです。

#### 3. MEUKF 出力の完全性を検証

```cpp
// output.pred_P や output.last_H の形式を
// mex_meukf_step_v2 の出力形式と一致させる
// これは MEUKF コアの実装確認が必要
```

#### 4. センサー前フレーム値の初期化を修正

```cpp
// 前フレーム値は呼び出しに応じて保持されるべき
// グローバル状態の一部として管理するか、
// 呼び出し元から渡される形式に変更
```

---

## 推奨事項

### **即座の対応**
1. **コミットを巻き戻す** `git revert` または `git reset`
2. 変更前のテスト結果を確認（基準コミットで `run_batch_10sets()` を再実行）

### **長期的な改善**
1. 座標系変換の単体テストを追加
2. MEUKF 直接呼び出しの検証テスト
3. MEX インターフェースの仕様書を整備
4. CI で回帰テストを自動化

---

## 戻す手順

### 方案 A：revert（推奨、履歴を保持）

```bash
cd /path/to/KalmanFilter

# 現在のコミットを確認
git log --oneline -5

# revert を実行（新しいコミットとして記録）
git revert 6207225a0ead9496f713c16ad8aa832f48d52137

# エディタが開いたら、revert メッセージを確認して保存
# :wq

# テスト実行
cd kalman
run_batch_10sets()  % MATLAB

# 結果を確認
compare_mex_matlab_detailed()

# push
git push origin phase6
```

### 方案 B：reset（履歴を修正、ローカルのみ推奨）

```bash
# 完全に巻き戻す（d3dab8c へ）
git reset --hard d3dab8c1887dd734b4641bc080436b266a47bf6c

# または、作業ディレクトリのみ戻す
git reset --soft d3dab8c1887dd734b4641bc080436b266a47bf6c

# テスト実行
cd kalman
clear mex
run_batch_10sets()  % MATLAB

# 確認後、新しいコミットで修正を開始
git add .
git commit -m "Revert: Restore previous MEUKF integration (before divergence)"
git push origin phase6
```

---

## 備考

現在のコミット `6207225` のメッセージは以下の通り：

> Fix filter divergence: Add NaN/Inf validation and fix MEUKF state update

しかし実際には、NaN/Inf 検証の追加ではなく、**根本的な座標系変換とインターフェースの再設計** が行われています。これらの変更は十分な検証なしに統合された可能性があります。

