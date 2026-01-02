# 重複ファイル現状確認レポート

**確認日**: 2025-01-XX  
**対象ドキュメント**: `CPP_DUPLICATE_FILES.md`

## 削除推奨ファイルの現状

### ✅ 削除済み（3ファイル）

#### 1. `MEUKF/meukf_core.cpp`
- **状態**: ✅ **削除済み**
- **確認**: `MEUKF/`ディレクトリは存在するが、中身は空
- **正規ファイル**: `src/MEUKF/meukf_core.cpp` が使用中

#### 2. `MEUKF/unified_filter.cpp`
- **状態**: ✅ **削除済み**
- **確認**: `MEUKF/`ディレクトリは存在するが、中身は空
- **正規ファイル**: `src/MEUKF/unified_filter.cpp` が使用中

#### 3. `MEUKF/meukf_types.hpp`
- **状態**: ✅ **削除済み**
- **確認**: `MEUKF/`ディレクトリは存在するが、中身は空
- **正規ファイル**: `Inc/MEUKF/meukf_types.hpp` が使用中

---

### ✅ 統合済み（1ファイル）

#### 4. `MEX/mex_type_conv.hpp`
- **状態**: ✅ **統合済み・削除済み**
- **確認**: ファイルは存在しない
- **統合先**: `Inc/MEX/mex_type_conversion.hpp` に統合
- **参考**: `CPP_IMPLEMENTATION_OVERVIEW.md` 70-72行目に記載
  - "`mex_type_conv.hpp`の内容を`mex_type_conversion.hpp`に統合"
  - "`MEX/mex_type_conv.hpp`を削除"

---

### ⚠️ まだ存在（1ファイル）

#### 5. `MEX/mex_meukf_step.cpp`
- **状態**: ⚠️ **まだ存在**（非推奨・deprecated）
- **場所**: `kalman/cpp/MEX/mex_meukf_step.cpp`
- **現状**:
  - ビルドシステムでビルドされている（`build_mex.m` 176行目）
  - しかし、`mexFunction`内で常にエラーを返す実装
  - ファイル内コメント（356-357行目）:
    ```cpp
    mexErrMsgIdAndTxt("mex_meukf_step:deprecated", 
        "mex_meukf_step_v2は統合済みです。mex_run_eskf内のdo_meukf_stepを使用してください。");
    ```
  - 実装コードは全てコメントアウトされている（95-353行目）
- **代替**: `mex_run_eskf`の`meukf_step`コマンドで代替可能
- **推奨**: 後方互換性を確認後、削除を検討

---

## 保持すべきファイルの現状

### ✅ 正しく配置されているファイル

#### 1. `Inc/MEX/mex_type_conversion.hpp`
- **状態**: ✅ **存在・使用中**
- **確認**: ファイルが存在し、複数のMEXファイルで使用されている

#### 2. 正規の実装ファイル
- ✅ `src/MEUKF/meukf_core.cpp` - 存在
- ✅ `src/MEUKF/unified_filter.cpp` - 存在
- ✅ `Inc/MEUKF/meukf_types.hpp` - 存在

---

## ディレクトリ構造の現状

```
kalman/cpp/
├── MEUKF/          # 空ディレクトリ（削除可能）
├── MEX/
│   ├── mex_meukf_step.cpp    # ⚠️ 非推奨（削除検討）
│   └── mex_run_eskf.cpp      # ✅ 使用中
├── src/
│   └── MEUKF/
│       ├── meukf_core.cpp      # ✅ 使用中
│       └── unified_filter.cpp  # ✅ 使用中
└── Inc/
    └── MEUKF/
        └── meukf_types.hpp     # ✅ 使用中
```

---

## 推奨アクション

### 即座に実行可能

1. **空ディレクトリの削除**
   ```bash
   rmdir kalman/cpp/MEUKF
   ```
   - `MEUKF/`ディレクトリは空なので削除可能

### 検討が必要

2. **`MEX/mex_meukf_step.cpp`の削除検討**
   - 後方互換性の確認が必要
   - 他のコードからの参照を確認（`grep -r "mex_meukf_step"`）
   - ビルドシステムから削除（`build_mex.m`）
   - 削除する場合は、関連ドキュメントも更新

---

## まとめ

| ファイル | ドキュメント記載 | 現状 | アクション |
|---------|----------------|------|-----------|
| `MEUKF/meukf_core.cpp` | 削除推奨 | ✅ 削除済み | - |
| `MEUKF/unified_filter.cpp` | 削除推奨 | ✅ 削除済み | - |
| `MEUKF/meukf_types.hpp` | 削除推奨 | ✅ 削除済み | - |
| `MEX/mex_type_conv.hpp` | 保持（必要） | ✅ 統合済み | - |
| `MEX/mex_meukf_step.cpp` | 検討が必要 | ⚠️ 非推奨・存在 | 削除検討 |
| `MEUKF/`ディレクトリ | - | ⚠️ 空ディレクトリ | 削除可能 |

**結論**: ドキュメントに記載されていた削除推奨ファイルの大部分は既に削除済み。残る課題は`mex_meukf_step.cpp`の削除検討と、空ディレクトリ`MEUKF/`の削除。

