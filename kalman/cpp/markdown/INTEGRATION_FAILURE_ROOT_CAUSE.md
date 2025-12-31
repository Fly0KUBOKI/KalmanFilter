# 統合失敗の根本原因分析 — 2025年12月31日

## 概要
コミット `477a5d4: 統合` 後、全シミュレーションで `NaN/Inf` エラーが発生。
原因は、**MEXバイナリの削除** と **センサーフィルタ初期化の無効化**。

---

## 失われた要素

### 1. 削除されたMEXバイナリ
```
❌ kalman/cpp/bin/mex_meukf_step_v2.mexw64
❌ kalman/cpp/bin/mex_sensor_filter.mexw64
```

**影響**:
- `mex_meukf_step_v2`: MEUKF（マルチ仮説フィルタ）の状態更新エンジン
- `mex_sensor_filter`: センサーアウトライア検出・スムージング機能

これらが無い場合、推定はフォールバック実装 or 失敗に陥る。

---

### 2. 無効化された初期化コード
#### ファイル: [kalman/run_batch_10sets.m](kalman/run_batch_10sets.m#L28-L35)

**削除前（動作版）**:
```matlab
if exist('mex_sensor_filter','file') == 3
    try
        mex_sensor_filter('reset_zero');
    catch
        try mex_sensor_filter('reset'); catch, end
    end
end
```

**削除後（失敗版）**:
```matlab
% mex_sensor_filterはmex_run_eskf('init')内で自動初期化されるため不要
% if exist('mex_sensor_filter','file') == 3
%     ...
% end
```

**根本原因**:
- センサーフィルタの状態がクリアされない
- 前回実行の異常状態が蓄積
- アウトライア検出が機能しない → NaN/Inf が出力される

**仮説の根拠**:
- `mex_run_eskf('init')` は ESKF初期化のみを行う
- センサーフィルタは **独立した** MEX状態機械
- 初期化呼び出しが無いと、MEX内部のフィルタ状態が不正

---

### 3. ビルドスクリプトのコメント化
#### ファイル: [kalman/cpp/build/build_mex.m](kalman/cpp/build/build_mex.m#L174-L185)

**削除前（動作版）**:
```matlab
meukf_core_cpp = fullfile(src_dir, 'MEUKF', 'meukf_core.cpp');
if exist('mex_meukf_step.cpp', 'file') && exist(meukf_core_cpp, 'file')
    if wants('mex_meukf_step') && build_single_mex(...)
        built_count = built_count + 1;
    end
end

if wants('mex_sensor_filter') && build_single_mex(...)
    built_count = built_count + 1;
end
```

**コメント化後（失敗版）**:
```matlab
% mex_meukf_step_v2とmex_sensor_filterはmex_run_eskfに統合済みのため、ビルド不要
% meukf_core_cpp = fullfile(src_dir, 'MEUKF', 'meukf_core.cpp');
% ...（全コメント化）
```

**根本原因**:
- 統合 (`mex_run_eskf` に機能統合) が **提案段階のまま** 実装されていない
- ビルドスクリプトは削除を前提としたが、**実装は未完成**
- MEXバイナリは古いままで、統合に対応していない

---

## データ分析による検証

### 復旧前の破損データ ([estimation_01.csv](kalman/Results/estimation_01.csv))
```
time,px,py,pz,vx,vy,vz,...
0,0,0,0,0,0,0,...
0.0025,0,0,0,0,0,0,...
0.005,0,0,0,0,0,0,...
...全て0で推定されていない...
```

**特徴**:
- 全ステップで位置・速度がゼロ
- センサー観測を無視している
- MEXが呼ばれていない、または初期化失敗を示唆

### 復旧方法
1. ✅ `mex_meukf_step_v2.mexw64` を復旧
2. ✅ `mex_sensor_filter.mexw64` を復旧
3. ✅ `run_batch_10sets.m` の初期化コード復旧
4. ✅ `build_mex.m` の個別ビルド設定復旧
5. ✅ `mex_run_eskf.mexw64` を安定版へ復旧

---

## 統合プロセスの問題点

| 問題 | 原因 | リスク |
|------|------|-------|
| MEX機能の統合計画が未実装 | コメント段階で止まっている | 削除に進む前に実装・検証が必要 |
| 初期化コードの削除 | 「自動」と仮定 | MEXの状態機械は明示的初期化が必須 |
| ビルドスクリプトの先読み | 存在しないターゲットのコメント化 | リバートできない状態に |
| 検証フェーズの省略 | バッチ実行で全数失敗まで気付かず | 単体テスト → 段階的統合が必須 |

---

## 再発防止策

### Phase 1: ビルドスクリプト保護（即実行）
```matlab
% build_mex.m に追加

% 統合チェック: 個別MEXビルドの削除禁止
% （コメント化も許さない。統合完了まで）

% [不許可なコメント化パターン]
% ❌ % meukf_core_cpp = ...  
% ❌ % if wants('mex_meukf_step') ...
% ❌ % if wants('mex_sensor_filter') ...

% [許可パターン]
% ✅ 統合完了時にのみ整理
```

### Phase 2: 初期化チェック（即実行）
```matlab
% run_batch_10sets.m に追加

% MEXセンサーフィルタ初期化チェック（削除禁止）
if exist('mex_sensor_filter', 'file') == 3
    % これは「不要」ではなく「必須」
    mex_sensor_filter('reset_zero');  % ← 削除すると即座にNaN発生
else
    warning('mex_sensor_filter not found: May cause NaN/Inf');
end
```

### Phase 3: 統合計画の検証プロセス（新規）

**メジャーな統合（複数MEX → 単一MEX）を行う場合**:

1. **計画フェーズ**:
   - 統合前：両方のMEXでテスト ✅
   - 統合後：統合版MEXでテスト ✅
   - 非回帰テスト（結果が同じ）✅

2. **実装フェーズ**:
   - C++ソースに統合コード追加（テスト付き）
   - 統合版MEXをビルド
   - 単体テスト実行

3. **統合フェーズ**:
   - 古いMEXをコメント化（削除ではなく）
   - 統合版MEXで `run_batch_10sets()` を実行
   - 結果が安定するまで並行検証

4. **確定フェーズ**:
   - 旧MEX削除 + ビルドスクリプト整理
   - リリース

### Phase 4: git commit メッセージのルール（新規）

```
❌ [統合] mex_meukf_step_v2と mex_sensor_filterを mex_run_eskfに統合
（実装なしで削除したコミット → 危険）

✅ [統合] mex_meukf_step_v2と mex_sensor_filterを mex_run_eskfに統合
   - mex_run_eskf.cpp に meukf_step() 処理を追加
   - meukf_core.cpp を mex_run_eskf.cpp にリンク
   - センサーフィルタ初期化を mex_run_eskf('init') に統合
   - 回帰テスト: run_batch_10sets() で 10/10 成功確認
   - 旧 mex_meukf_step_v2, mex_sensor_filter を削除
```

---

## チェックリスト

### 統合を行う前に確認すること

- [ ] 統合対象のMEXが現在も動作しているか（単体テスト）
- [ ] 統合後のC++コードを **先に実装** したか
- [ ] 統合版MEXを別名（例: `mex_run_eskf_v2`）でビルドしたか
- [ ] 統合前後のMEXで **同一入出力** をテストしたか
- [ ] `run_batch_10sets()` で 10/10 成功したか
- [ ] センサーフィルタなどの初期化が **明示的に呼ばれているか**
- [ ] 初期化コードが削除・コメント化されていないか
- [ ] git statusで **想定外の削除** が無いか

### 失敗時の緊急対応

```bash
# 現在の問題を特定
git diff

# バイナリが削除されていないか確認
git status | grep deleted.*mexw64

# 初期化コードが削除されていないか確認
git diff kalman/run_*.m | grep -E "^\-.*reset|^\-.*init"

# 復旧（緊急時）
git restore kalman/cpp/bin/*.mexw64
git restore kalman/run_batch_10sets.m
git restore kalman/cpp/build/build_mex.m
```

---

## まとめ

| 要素 | 状態 | 復旧方法 |
|------|------|---------|
| MEX削除 | ❌ コミット後の削除 | git restore |
| 初期化削除 | ❌ コメント化 | git restore |
| ビルド設定 | ❌ コメント化 | git restore |
| 実装の統合 | ❌ **未実装** | Phase 3参照 |

**重要**: 統合の **考え方** は正しいが、**実行順序** が逆だった。
- ❌ 削除 → 統合
- ✅ 統合実装 → テスト → 削除

---

**作成日**: 2025年12月31日  
**対象プロジェクト**: KalmanFilter (phase6)  
**関連ファイル**: 
- [kalman/run_batch_10sets.m](kalman/run_batch_10sets.m)
- [kalman/cpp/build/build_mex.m](kalman/cpp/build/build_mex.m)
- [kalman/cpp/bin/mex_*.mexw64](kalman/cpp/bin/)
