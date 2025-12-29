# ソースコードがないバイナリの調査

## 調査結果

### binフォルダに存在するがソースがないバイナリ

| バイナリ | ソースファイル | 状態 | 推奨アクション |
|---------|--------------|------|--------------|
| `mex_eskf_reset.mexw64` | ❌ なし | 古いバイナリ | 削除を検討 |
| `mex_eskf_run.mexw64` | ✅ `mex_run_eskf.cpp` | 名前不一致 | 名前を統一するか削除 |
| `mex_eskf_core.mexw64` | ✅ `mex_eskf_core.cpp` | ビルドスキップ | 使用状況を確認 |

### MEXフォルダに存在するバイナリ（古いビルド残骸）

| バイナリ | ソースファイル | 状態 | 推奨アクション |
|---------|--------------|------|--------------|
| `mex_eskf_predict_postprocess.mexw64` | ❌ 削除済み | ソースが削除された | バイナリも削除 |
| `mex_eskf_update_postprocess.mexw64` | ❌ 削除済み | ソースが削除された | バイナリも削除 |
| `mex_eskf_zupt.mexw64` | ❌ 削除済み | ソースが削除された | バイナリも削除 |
| `mex_eskf_reset.mexw64` | ❌ なし | 古いバイナリ | 削除を検討 |

## 詳細調査

### mex_eskf_reset.mexw64

**状況:**
- binフォルダとMEXフォルダの両方に存在
- ソースコード（.cpp）が存在しない
- `build_mex.m`にも記載なし

**使用状況:**
- `GRADUAL_MEX_MIGRATION_PLAN.md`で言及されているが、実際の使用箇所は見つからない
- `mex_filter_management`が代わりに使用されている可能性が高い

**推奨:**
- 使用されていない場合は削除
- 必要であれば`mex_filter_management`の機能として統合

### mex_eskf_run.mexw64

**状況:**
- binフォルダに存在
- ソースは`mex_run_eskf.cpp`として存在
- 名前が不一致（`mex_eskf_run` vs `mex_run_eskf`）

**推奨:**
- 古いバイナリの可能性が高い
- `mex_run_eskf.mexw64`が正しい名前
- 削除を検討

### mex_eskf_core.mexw64

**状況:**
- binフォルダに存在
- ソースは`mex_eskf_core.cpp`として存在
- `build_mex.m`で「Skip locked legacy target」としてスキップされている

**使用状況:**
- コードベース内で使用箇所が見つからない
- `mex_eskf_core_v2.cpp`が存在するが、これもビルド対象外

**推奨:**
- 使用されていない場合は削除
- 必要であれば`mex_eskf_core_v2`をビルド対象に追加

### mex_quaternion_lib.mexw64

**状況:**
- binフォルダに存在
- ソースは`mex_quaternion_lib.cpp`として存在
- `build_mex.m`で「locked/skipped」としてスキップされている

**使用状況:**
- `mex_matlab_helpers.m`で使用されている
- `GRADUAL_MEX_MIGRATION_PLAN.md`で言及

**推奨:**
- 使用されている場合はビルド対象に追加
- または`mex_kalman_compute`や`mex_common_lib`に統合

### mex_kalman_compute.mexw64

**状況:**
- binフォルダに存在
- ソースは`mex_kalman_compute.cpp`として存在
- `build_mex.m`に記載なし

**使用状況:**
- コードベース内で使用箇所が見つからない

**推奨:**
- 使用されていない場合は削除
- またはビルド対象に追加

### mex_ukf_update_minimal.mexw64

**状況:**
- binフォルダに存在
- ソースは`mex_ukf_update_minimal.cpp`として存在
- `build_mex.m`に記載なし

**使用状況:**
- `mex_ukf_update.cpp`が存在し、こちらがビルド対象

**推奨:**
- 重複機能の可能性
- 使用されていない場合は削除

### mex_meukf_step.mexw64

**状況:**
- binフォルダに存在
- ソースは`mex_meukf_step.cpp`として存在
- `build_mex.m`で`mex_meukf_step_v2`としてビルド

**使用状況:**
- `mex_meukf_step_v2.mexw64`が正しい出力名

**推奨:**
- 古いバイナリの可能性
- 削除を検討

## 削除推奨リスト

以下のバイナリは削除を検討：

1. `mex_eskf_reset.mexw64` (bin, MEX両方)
2. `mex_eskf_run.mexw64` (bin)
3. `mex_eskf_core.mexw64` (bin) - 使用されていない場合
4. `mex_meukf_step.mexw64` (bin) - `mex_meukf_step_v2`が正しい
5. `mex_ukf_update_minimal.mexw64` (bin) - 重複の可能性
6. `mex_kalman_compute.mexw64` (bin) - 使用されていない場合
7. MEXフォルダ内の`.mexw64`ファイル（すべて古いビルド残骸）

## ビルド対象に追加を検討

以下のファイルはソースが存在し、使用されている可能性があるため、ビルド対象に追加を検討：

1. `mex_quaternion_lib.cpp` - `mex_matlab_helpers.m`で使用
2. `mex_kalman_compute.cpp` - 使用状況を確認
3. `mex_common_lib.cpp` - 使用状況を確認
4. `mex_filter_utils.cpp` - 使用状況を確認
5. `mex_eskf_helper.cpp` - 使用状況を確認
6. `mex_eskf_core_v2.cpp` - 使用状況を確認



