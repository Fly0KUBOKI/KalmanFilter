# MEXファイル統合分析

## 現在の状況

### 3つのMEXファイルが存在

1. **`mex_run_eskf.mexw64`** - メインエントリーポイント
2. **`mex_sensor_filter.mexw64`** - 初期化用（MATLABから直接呼び出し）
3. **`mex_meukf_step_v2.mexw64`** - センサー更新処理（`mex_run_eskf`から`mexCallMATLAB`経由で呼び出し）

## 統合できない理由（現在）

### 1. `mex_meukf_step_v2`が統合されていない理由

**現在の実装:**
```cpp
// mex_run_eskf_sensor_updates.hpp (行407-410)
// mex_meukf_step_v2 呼び出し（後でMEUKFCore::stepに置き換え予定）
mxArray* prhs_m[3] = {state_s, sensor_data, mex_params};
mxArray* plhs_m[3];
if (mexCallMATLAB(3, plhs_m, 3, prhs_m, "mex_meukf_step_v2") == 0) {
```

**統合されていない理由:**
1. **型変換の複雑さ**: MATLAB構造体 ↔ C++構造体の変換が必要
2. **座標系変換**: MATLAB (column-major) ↔ C++ (row-major) の変換が必要
3. **段階的移行**: 既存の動作を保証しながら移行するため、`mexCallMATLAB`経由で呼び出している
4. **コメントに記載**: 「後でMEUKFCore::stepに置き換え予定」と明記されている

**統合可能か？**
✅ **はい、統合可能です**

`mex_meukf_step.cpp`の実装を見ると、以下の変換を行っています：
- `matlab_to_state()`: MATLAB構造体 → C++構造体
- `state_to_matlab()`: C++構造体 → MATLAB構造体

これらの変換ロジックを`mex_run_eskf_sensor_updates.hpp`に統合すれば、`MEUKFCore::step()`を直接呼び出せます。

### 2. `mex_sensor_filter`が統合されていない理由

**現在の使用状況:**
```matlab
% run_batch_10sets.m (行28-33)
if exist('mex_sensor_filter','file') == 3
    try
        mex_sensor_filter('reset_zero');
    catch
        try mex_sensor_filter('reset'); catch, end
    end
end
```

**統合されていない理由:**
1. **初期化タイミング**: `mex_run_eskf('init')`の前に呼び出される
2. **グローバル状態**: `mex_sensor_filter`はグローバルなセンサーフィルターライブラリを管理
3. **独立性**: 他の用途でも使用される可能性を考慮

**統合可能か？**
✅ **はい、統合可能です**

`mex_run_eskf('init')`内で`mex_sensor_filter('reset_zero')`を呼び出すように変更すれば、MATLABコードからの直接呼び出しを削除できます。

## 統合のメリット

### 1. パフォーマンス向上
- `mexCallMATLAB`のオーバーヘッドを削減
- 型変換の最適化が可能
- メモリコピーの削減

### 2. コードの簡素化
- MEXファイル数が3つ → 1つに削減
- 依存関係の簡素化
- ビルド時間の短縮

### 3. 保守性の向上
- コードが1箇所に集約
- デバッグが容易
- テストが簡素化

## 統合方法

### Phase 1: `mex_meukf_step_v2`の統合

**手順:**
1. `mex_meukf_step.cpp`の`matlab_to_state()`と`state_to_matlab()`を`mex_run_eskf_sensor_updates.hpp`に統合
2. `mexCallMATLAB`の代わりに`MEUKFCore::step()`を直接呼び出し
3. 座標系変換を正確に実装
4. テスト実行

**実装例:**
```cpp
// mex_run_eskf_sensor_updates.hpp内で
meukf::MEUKFInput input;
meukf::MEUKFOutput output;

// MATLAB構造体 → C++構造体の変換
matlab_to_meukf_input(state_s, sensor_data, mex_params, input);

// MEUKFCore::step()を直接呼び出し
meukf::MEUKFCore::step(input, output);

// C++構造体 → MATLAB構造体の変換
meukf_output_to_matlab(output, new_state, dbg_out);
```

### Phase 2: `mex_sensor_filter`の統合

**手順:**
1. `mex_run_eskf('init')`内で`g_filter_lib`を初期化
2. MATLABコードから`mex_sensor_filter`の呼び出しを削除
3. テスト実行

**実装例:**
```cpp
// mex_run_eskf_impl.hpp内のdo_init()で
inline uint64_t do_init(const mxArray* obs, double static_time, double dt) {
    // センサーフィルターライブラリの初期化
    g_filter_lib.reset_zero();  // または適切な初期化メソッド
    
    ESKFState* s = initialize_eskf_from_matlab(obs, static_time, dt);
    // ...
}
```

## 統合のリスク

### 1. 型変換の正確性
- 座標系変換の誤りが発生する可能性
- 十分なテストが必要

### 2. 互換性
- 既存のMATLABコードとの互換性を保証
- 段階的な移行が必要

### 3. デバッグの難しさ
- 統合後は問題の切り分けが難しくなる可能性
- 十分なログ出力が必要

## 推奨される統合手順

### Step 1: 準備
1. 現在のテスト結果をベースラインとして記録
2. `mex_meukf_step.cpp`の変換ロジックを詳細に分析
3. 統合計画を作成

### Step 2: `mex_meukf_step_v2`の統合
1. 変換関数を`mex_run_eskf_sensor_updates.hpp`に追加
2. `mexCallMATLAB`を`MEUKFCore::step()`に置き換え
3. 単体テスト実行
4. バッチテスト実行（`run_batch_10sets()`）
5. 結果をベースラインと比較

### Step 3: `mex_sensor_filter`の統合
1. `mex_run_eskf('init')`内で初期化を追加
2. MATLABコードから`mex_sensor_filter`の呼び出しを削除
3. テスト実行

### Step 4: クリーンアップ
1. `mex_meukf_step.cpp`を削除またはアーカイブ
2. `mex_sensor_filter.cpp`を削除またはアーカイブ
3. `build_mex.m`から該当エントリを削除
4. ドキュメントを更新

## 結論

**統合できない理由は技術的な制約ではなく、単にまだ実装されていないだけです。**

統合は可能であり、以下のメリットがあります：
- ✅ パフォーマンス向上
- ✅ コードの簡素化
- ✅ 保守性の向上

ただし、慎重な実装と十分なテストが必要です。


