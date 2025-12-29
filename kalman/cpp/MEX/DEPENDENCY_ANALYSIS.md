# MEXファイル間の依存関係分析

## エラーから判明した依存関係

エラーログより：
```
次を使用中のエラー: mex_eskf_predict_postprocess
関数 'mex_quaternion_lib' (タイプ'char' の入力引数) が未定義です。
```

**結論**: `mex_eskf_predict_postprocess` → `mex_quaternion_lib` の依存関係が存在

## 問題点

1. **`mex_eskf_predict_postprocess.cpp`のソースファイルが存在しない**
   - binディレクトリには`mex_eskf_predict_postprocess.mexw64`が存在
   - しかし、MEXディレクトリにソースファイルがない
   - 古いバイナリが`mex_quaternion_lib`を呼び出している可能性

2. **`mex_quaternion_lib`がコンパイルされていない**
   - build_mex.mで`locked/skipped`とされていた
   - しかし、実際には使用されている

## 確認された依存関係（mexCallMATLAB使用）

### mex_run_eskf.cpp の依存関係
- `mex_adaptive_predict` (line 112)
- `mex_eskf_predict_postprocess` (line 140) → **これが`mex_quaternion_lib`を呼び出す**
- `mex_eskf_sensor_updates_full` (line 192, 253)
- `mex_filter_management` (line 278, 315)
- `mex_eskf_zupt` (line 377)
- `mex_eskf_constructor` (line 403)

### mex_eskf_do_update.cpp の依存関係
- `mex_sensor_filter` (line 64, 202)
- `mex_meukf_step_v2` (line 189)
- `mex_eskf_update_postprocess` (line 224)

### mex_eskf_sensor_updates_full.cpp の依存関係
- `mex_sensor_preprocessor` (line 72, 173, 272, 372)
- `mex_eskf_do_update` (line 109, 209, 307, 411)

## 依存関係ツリー

```
mex_run_eskf
├─ mex_adaptive_predict
├─ mex_eskf_predict_postprocess
│  └─ mex_quaternion_lib ⚠️ (依存関係が判明)
├─ mex_eskf_sensor_updates_full
│  ├─ mex_sensor_preprocessor
│  └─ mex_eskf_do_update
│     ├─ mex_sensor_filter
│     ├─ mex_meukf_step_v2
│     └─ mex_eskf_update_postprocess
├─ mex_filter_management
├─ mex_eskf_zupt
└─ mex_eskf_constructor
```

## 必要な対応

1. **`mex_quaternion_lib`をコンパイル対象に追加** ✅ (既に実施済み)
2. **`mex_eskf_predict_postprocess.cpp`のソースファイルを復元** ✅ (完了)
   - gitコミット `11ee30f` から復元済み
   - `mex_quaternion_lib`を`mexCallMATLAB`で呼び出していることを確認（78行目、128行目）
3. **`mex_eskf_update_postprocess.cpp`のソースファイルを復元** ✅ (完了)
   - gitコミット `9a45d06` から復元済み
4. **`mex_meukf_step.cpp`のソースファイルを復元** ✅ (完了)
   - gitコミット `616978f` から復元済み

## ビルド順序の推奨

依存関係を考慮したビルド順序：
1. `mex_quaternion_lib` (依存なし)
2. `mex_sensor_preprocessor` (依存なし)
3. `mex_sensor_filter` (依存なし)
4. `mex_adaptive_predict` (依存なし)
5. `mex_meukf_step_v2` (依存なし)
6. `mex_eskf_constructor` (依存なし)
7. `mex_filter_management` (依存なし)
8. `mex_eskf_zupt` (依存なし)
9. `mex_eskf_predict_postprocess` (依存: `mex_quaternion_lib`)
10. `mex_eskf_do_update` (依存: `mex_sensor_filter`, `mex_meukf_step_v2`)
11. `mex_eskf_update_postprocess` (依存: 未確認)
12. `mex_eskf_sensor_updates_full` (依存: `mex_sensor_preprocessor`, `mex_eskf_do_update`)
13. `mex_run_eskf` (依存: すべて)
