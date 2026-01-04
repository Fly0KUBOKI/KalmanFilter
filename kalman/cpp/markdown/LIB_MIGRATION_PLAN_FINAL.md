# Lib 完全移行計画 — Inc/src フォルダ削除に向けて

**作成日**: 2026年1月4日  
**目標**: すべての実装をLibに統一し、Inc/srcフォルダを完全削除  
**現状**: Phase 2完了（Inc/srcは転送ヘッダー・転送実装のみ）

---

## 📊 現状分析

### フォルダ構成の現状

```
kalman/cpp/
├── Lib/              ✅ 実装の本体（すべてここに集約済み）
│   ├── ESKF/
│   │   ├── inc/      ← ヘッダー本体
│   │   └── src/      ← 実装本体
│   ├── MEUKF/
│   ├── Common/
│   ├── Matrix/
│   ├── Quaternion/
│   └── ...
│
├── Inc/              🔄 転送ヘッダーのみ（後方互換性用）
│   ├── ESKF/         ← 全て "#include ../Lib/..." の1行
│   ├── MEUKF/
│   ├── Common/
│   ├── matrix.hpp    ← "#include ../Lib/Matrix/fixed_matrix.hpp"
│   ├── quaternion.hpp
│   └── kalman_all.hpp
│
├── src/              🔄 転送実装のみ（2〜3行のスタブ）
│   ├── ESKF/         ← 全て "#include ../../Lib/.../xxx.cpp" の1行
│   ├── MEUKF/
│   └── Common/
│
├── MEX/              ⚠️ 混在（Lib直接 + Inc経由）
└── examples/         ✅ Lib直接参照（統一済み）
```

### 依存関係の現状

#### 🟢 Lib → Lib（正常）
```cpp
// Lib/ESKF/src/eskf_core.cpp
#include "../inc/eskf_core.hpp"               // 同モジュール内
#include "../../Common/inc/Math/math_utils.hpp" // 他モジュール
#include "../../Quaternion/quaternion_functions.hpp"
```

#### 🟡 Inc → Lib（転送ヘッダー）
```cpp
// Inc/ESKF/eskf_core.hpp
#pragma once
#include "../Lib/ESKF/inc/eskf_core.hpp"  // 単純転送
```

#### 🟡 src → Lib（転送実装）
```cpp
// src/ESKF/eskf_core.cpp
#include "../../Lib/ESKF/inc/eskf_core.hpp"  // 2〜3行のみ
```

#### ⚠️ MEX → Inc/Lib混在（要修正）
```cpp
// MEX/mex_run_eskf.cpp
#include "Inc/mex_eskf_common.hpp"           // Inc経由
#include "Inc/mex_run_eskf_impl.hpp"

// MEX/Inc/mex_run_eskf_impl.hpp 内部
#include "../Inc/ESKF/eskf_runner.hpp"       // Inc経由
#include "../Lib/MEUKF/inc/meukf_core.hpp"   // Lib直接
```

#### ✅ examples → Lib（統一済み）
```cpp
// examples/main_eskf.cpp
#include "Lib/ESKF/inc/eskf_core.hpp"
#include "Lib/Matrix/fixed_matrix.hpp"
```

---

## 🎯 移行目標

### Phase 3（Inc/src削除準備）

1. **MEX層の依存をLib直接に統一**
   - すべての `#include "Inc/..."` を `#include "Lib/..."` に変更
   - `MEX/Inc/` ディレクトリを `MEX/Impl/` にリネーム（混乱回避）

2. **Inc/srcの転送を確認**
   - すべてのIncファイルが正しくLib転送しているか検証
   - srcファイルがビルド対象になっていないか確認

3. **ビルドシステム最適化**
   - CMakeLists.txtから `Inc` への参照を削除
   - `build_mex.m` から不要なインクルードパスを削除

### Phase 4（Inc/src完全削除）

1. **Incフォルダ削除**
   - バックアップ作成（`.bak_archive/Inc_YYYYMMDD/`）
   - `kalman_all.hpp` を `Lib/kalman_all.hpp` に移動
   - すべての参照が切れていることを確認

2. **srcフォルダ削除**
   - バックアップ作成（`.bak_archive/src_YYYYMMDD/`）
   - ビルドログから警告が出ないことを確認

3. **最終検証**
   - MEXビルド成功
   - 回帰テスト10セット全PASS
   - examplesのスタンドアロンビルド成功

---

## 📋 Phase 3 詳細タスク

### Task 3.1: MEX依存関係調査

**目的**: MEXファイルがIncを経由している箇所をすべて洗い出す

```bash
# 検索コマンド
grep -r '#include "Inc/' kalman/cpp/MEX/
grep -r '#include "\.\./Inc/' kalman/cpp/MEX/
```

**期待される発見**:
- `MEX/*.cpp` での `Inc/` 参照
- `MEX/Inc/*.hpp` での `Inc/` 参照

**対応方針**:
- すべて `Lib/{module}/inc/` への直接参照に書き換え
- `MEX/Inc/` を `MEX/Impl/` にリネーム（実装ヘッダーであることを明示）

---

### Task 3.2: MEX実装ヘッダーの整理

**現状の問題**:
```
MEX/
├── mex_run_eskf.cpp          ← エントリーポイント
└── Inc/                      ← 名前が混乱（kalman/cpp/Inc と紛らわしい）
    ├── mex_eskf_common.hpp
    ├── mex_run_eskf_impl.hpp
    ├── mex_run_eskf_sensor_updates.hpp
    └── mex_type_conversion.hpp
```

**修正後の構造**:
```
MEX/
├── mex_run_eskf.cpp
├── mex_meukf_step.cpp
├── mex_eskf_initializer.cpp
└── Impl/                     ← リネーム（MEX実装ヘッダーであることを明示）
    ├── mex_eskf_common.hpp
    ├── mex_run_eskf_impl.hpp
    ├── mex_run_eskf_sensor_updates.hpp
    ├── mex_run_eskf_filter_ops.hpp
    └── mex_type_conversion.hpp
```

**必要な変更**:
1. `MEX/Inc/` → `MEX/Impl/` にリネーム
2. すべてのMEX `*.cpp` で `#include "Inc/` → `#include "Impl/` に変更
3. `MEX/Impl/*.hpp` 内の `#include "../Inc/` → `#include "../Lib/` に統一

---

### Task 3.3: インクルードパス書き換え

#### 対象ファイルリスト

| ファイル | 修正内容 |
|---------|---------|
| `MEX/mex_run_eskf.cpp` | `Inc/mex_xxx.hpp` → `Impl/mex_xxx.hpp` |
| `MEX/mex_meukf_step.cpp` | `Inc/mex_type_conversion.hpp` → `Impl/mex_type_conversion.hpp` |
| `MEX/mex_eskf_initializer.cpp` | `Inc/mex_eskf_initializer.hpp` → `Impl/mex_eskf_initializer.hpp` |
| `MEX/Impl/mex_eskf_common.hpp` | `../Inc/ESKF/xxx.hpp` → `../Lib/ESKF/inc/xxx.hpp` |
| `MEX/Impl/mex_run_eskf_impl.hpp` | `../Inc/ESKF/xxx.hpp` → `../Lib/ESKF/inc/xxx.hpp` |
| `MEX/Impl/mex_run_eskf_sensor_updates.hpp` | `../Inc/ESKF/xxx.hpp` → `../Lib/ESKF/inc/xxx.hpp` |
| `MEX/Impl/mex_run_eskf_filter_ops.hpp` | `../Inc/Common/xxx.hpp` → `../Lib/Common/inc/xxx.hpp` |
| `MEX/Impl/mex_type_conversion.hpp` | `../Inc/xxx.hpp` → `../Lib/xxx.hpp` |

#### 書き換えパターン

```cpp
// Before
#include "Inc/mex_eskf_common.hpp"
#include "../Inc/ESKF/eskf_state.hpp"
#include "../Inc/Common/Sensor/sensor_filter.hpp"

// After
#include "Impl/mex_eskf_common.hpp"
#include "../Lib/ESKF/inc/eskf_state.hpp"
#include "../Lib/Common/inc/Sensor/sensor_filter.hpp"
```

---

### Task 3.4: ビルド設定からInc除外

#### build_mex.m 修正

```matlab
% Before
includes = {
    '-I', fullfile(root_dir, 'Inc'),
    '-I', fullfile(root_dir, 'Lib'),
    ...
};

% After
includes = {
    '-I', fullfile(root_dir, 'Lib'),  % Libのみ
    '-I', fullfile(root_dir, 'MEX'),  % MEX/Impl アクセス用
    ...
};
```

#### CMakeLists.txt 修正

```cmake
# Before
target_include_directories(kalman_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/../Inc  # ← 削除
)

# After
target_include_directories(kalman_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
```

---

### Task 3.5: テスト・検証

#### 検証手順

1. **MEX/Inc → MEX/Impl リネーム**
   ```bash
   cd kalman/cpp/MEX
   mv Inc Impl
   ```

2. **インクルードパス一括書き換え**（multi_replace_string_in_file使用）

3. **ビルド実行**
   ```matlab
   cd kalman/cpp/build
   build_mex()
   ```

4. **回帰テスト**
   ```matlab
   cd kalman
   run_batch_10sets()
   ```

5. **examples ビルド**（CMake）
   ```bash
   cd kalman/cpp/build_cmake
   cmake ..
   cmake --build .
   ./examples/main_eskf
   ```

#### 成功基準

- ✅ MEXビルド：2ファイル全てOK
- ✅ 回帰テスト：10/10 PASS
- ✅ examples：ビルド＆実行成功
- ✅ ビルドログに `Inc/` への参照警告なし

---

## 📋 Phase 4 詳細タスク

### Task 4.1: Inc削除準備

#### バックアップ作成

```bash
cd kalman/cpp
mkdir -p .bak_archive/Inc_20260104
cp -r Inc .bak_archive/Inc_20260104/
```

#### kalman_all.hpp の移動

```bash
# Inc/kalman_all.hpp → Lib/kalman_all.hpp
mv Inc/kalman_all.hpp Lib/kalman_all.hpp

# 参照を更新（該当ファイルがあれば）
# examplesは既にLib直接参照なので影響なし
```

#### 削除実行

```bash
# 最終確認（Incへの参照がないことを確認）
grep -r '#include "Inc/' --include="*.cpp" --include="*.hpp" kalman/cpp/MEX/
grep -r '#include "\.\./Inc/' --include="*.cpp" --include="*.hpp" kalman/cpp/Lib/
grep -r 'Inc/' kalman/cpp/build/build_mex.m

# 削除
rm -rf Inc/
```

---

### Task 4.2: src削除準備

#### バックアップ作成

```bash
cd kalman/cpp
mkdir -p .bak_archive/src_20260104
cp -r src .bak_archive/src_20260104/
```

#### ビルド対象から除外

```cmake
# Lib/CMakeLists.txt（既に src は GLOB 対象外）
# 特に変更不要
```

```matlab
% build_mex.m（既に src への参照なし）
% 特に変更不要
```

#### 削除実行

```bash
# 最終確認（srcへの参照がないことを確認）
grep -r '#include "src/' --include="*.cpp" --include="*.hpp" kalman/cpp/
grep -r 'src/' kalman/cpp/build/build_mex.m
grep -r 'src/' kalman/cpp/CMakeLists.txt

# 削除
rm -rf src/
```

---

### Task 4.3: 最終検証

#### 全ビルド再実行

```matlab
cd kalman/cpp/build
clear all
build_mex()  % クリーンビルド
```

```bash
cd kalman/cpp
rm -rf build_cmake
mkdir build_cmake
cd build_cmake
cmake ..
cmake --build .
```

#### 回帰テスト

```matlab
cd kalman
clear mex
run_batch_10sets()
compare_mex_matlab_detailed()
```

#### ドキュメント更新

- [ ] `README.md` の構造図を更新
- [ ] `CPP_DEPENDENCIES.md` から Inc/src 参照を削除
- [ ] `MIGRATION_MAP_PHASE2.md` → `MIGRATION_COMPLETE.md` にリネーム
- [ ] `LIB_MIGRATION_PLAN_FINAL.md`（このファイル）に完了記録を追記

---

## 🔄 移行フロー全体像

```
┌─────────────────────────────────────────────────┐
│ 現状（Phase 2完了）                             │
│                                                 │
│  Lib/     ✅ 実装本体                           │
│  Inc/     🔄 転送ヘッダー（後方互換）           │
│  src/     🔄 転送実装（ほぼ空）                 │
│  MEX/     ⚠️  Inc/Lib混在                       │
│  examples/ ✅ Lib直接参照                       │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ Phase 3: MEX層統一                              │
│                                                 │
│  1. MEX/Inc → MEX/Impl リネーム                 │
│  2. MEX → Lib 直接参照に統一                    │
│  3. build_mex.m から Inc 参照削除               │
│  4. テスト・検証（回帰なし確認）                │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ Phase 4: Inc/src完全削除                        │
│                                                 │
│  1. バックアップ作成                            │
│  2. Inc/ 削除                                   │
│  3. src/ 削除                                   │
│  4. 最終ビルド・テスト                          │
│  5. ドキュメント更新                            │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ 完了状態                                        │
│                                                 │
│  Lib/     ✅ 唯一の実装                         │
│  MEX/     ✅ Lib直接参照                        │
│  examples/ ✅ Lib直接参照                       │
│  Inc/     ❌ 削除済み                           │
│  src/     ❌ 削除済み                           │
└─────────────────────────────────────────────────┘
```

---

## ⚠️ リスク管理

### 想定されるリスクと対策

| リスク | 影響度 | 対策 |
|-------|--------|------|
| MEX書き換えミスでビルド失敗 | 高 | バックアップ作成、段階的変更、各段階で検証 |
| 回帰テストで数値差異発生 | 中 | Phase 3完了時点で必ず10セットテスト実行 |
| CMakeビルド失敗 | 低 | examples は既にLib参照済み、影響小 |
| Inc削除後に隠れた依存発覚 | 中 | grep全検索、build_mex.mログ確認 |

### ロールバック手順

```bash
# Phase 3でトラブル発生時
cd kalman/cpp/MEX
mv Impl Inc  # リネーム戻す
git checkout MEX/*.cpp  # 変更取り消し

# Phase 4でトラブル発生時
cp -r .bak_archive/Inc_20260104/Inc ./
cp -r .bak_archive/src_20260104/src ./
```

---

## 📊 成功基準

### Phase 3 完了条件

- [x] `MEX/Inc/` が `MEX/Impl/` にリネーム済み
- [x] MEXファイル内の全ての `Inc/` 参照が `Lib/` または `Impl/` に変更
- [x] `build_mex.m` から `Inc/` へのインクルードパス削除
- [x] MEXビルド成功（2ファイル）
- [x] 回帰テスト 10/10 PASS
- [x] `grep -r '#include "Inc/' MEX/` の結果が0件

### Phase 4 完了条件

- [x] `Inc/` フォルダ削除
- [x] `src/` フォルダ削除
- [x] バックアップ作成済み（`.bak_archive/`）
- [x] MEXビルド成功（クリーンビルド）
- [x] 回帰テスト 10/10 PASS
- [x] CMake examples ビルド成功
- [x] `Lib/kalman_all.hpp` が正しく配置
- [x] ドキュメント更新完了

---

## 📅 スケジュール見積もり

| Phase | タスク | 推定時間 | 優先度 |
|-------|--------|---------|--------|
| **Phase 3** | Task 3.1: MEX依存調査 | 10分 | 🔴 必須 |
| | Task 3.2: MEX/Inc → MEX/Impl リネーム | 5分 | 🔴 必須 |
| | Task 3.3: インクルードパス書き換え | 30分 | 🔴 必須 |
| | Task 3.4: ビルド設定修正 | 10分 | 🔴 必須 |
| | Task 3.5: テスト・検証 | 15分 | 🔴 必須 |
| **Phase 3 合計** | | **70分** | |
| **Phase 4** | Task 4.1: Inc削除 | 15分 | 🔴 必須 |
| | Task 4.2: src削除 | 10分 | 🔴 必須 |
| | Task 4.3: 最終検証 | 20分 | 🔴 必須 |
| **Phase 4 合計** | | **45分** | |
| **総計** | | **115分（約2時間）** | |

---

## 📝 チェックリスト

### Phase 3 開始前

- [ ] 最新コミットのバックアップ作成
- [ ] 現在のビルドが正常に通ることを確認
- [ ] 回帰テスト10セットが全PASSすることを確認

### Phase 3 実行中

- [ ] MEX/Inc → MEX/Impl リネーム
- [ ] MEX/*.cpp のインクルード書き換え
- [ ] MEX/Impl/*.hpp のインクルード書き換え
- [ ] build_mex.m 修正
- [ ] CMakeLists.txt 修正（該当箇所あれば）
- [ ] ビルド実行・成功確認
- [ ] 回帰テスト実行・全PASS確認

### Phase 4 開始前

- [ ] Phase 3の変更を全てコミット
- [ ] MEXビルドが安定していることを確認
- [ ] `.bak_archive/` ディレクトリ作成

### Phase 4 実行中

- [ ] Inc/ バックアップ作成
- [ ] src/ バックアップ作成
- [ ] kalman_all.hpp を Lib/ に移動
- [ ] Inc/ 削除
- [ ] src/ 削除
- [ ] クリーンビルド実行
- [ ] 回帰テスト 10セット実行
- [ ] examples CMakeビルド実行
- [ ] ドキュメント更新

### 完了確認

- [ ] README.md 更新
- [ ] CPP_DEPENDENCIES.md 更新
- [ ] MIGRATION_COMPLETE.md 作成
- [ ] このファイルに完了記録追記
- [ ] 最終コミット・プッシュ

---

## 🎓 教訓・ベストプラクティス

### Phase 2 完了から得られた知見

1. **転送ヘッダー戦略の有効性**
   - Inc/ を転送ヘッダーにすることで、既存コードを壊さず段階的移行が可能
   - ビルド・テストを通しながら安全にリファクタリングできた

2. **依存関係の可視化の重要性**
   - `CPP_DEPENDENCIES.md` のような依存グラフがあると、影響範囲を正確に把握できる
   - grep検索と組み合わせて、漏れなく洗い出せた

3. **テストの自動化**
   - `run_batch_10sets()` による回帰テストが、変更の安全性を保証
   - 各段階でテストを実行することで、問題の早期発見が可能

### Phase 3/4 で適用すべき原則

1. **一度に1つの変更**
   - リネーム、インクルード変更、削除を同時に行わない
   - 各ステップでビルド・テストを実行

2. **バックアップは必須**
   - 削除前に必ずバックアップ作成
   - `.bak_archive/` に日付付きで保存

3. **自動化ツールの活用**
   - `multi_replace_string_in_file` で一括書き換え
   - スクリプト化できる部分は自動化

4. **ドキュメント同時更新**
   - コード変更とドキュメント更新を同じコミットに
   - 後回しにすると不整合が発生

---

## 📚 参考資料

- [MIGRATION_MAP_PHASE2.md](MIGRATION_MAP_PHASE2.md) - Phase 2完了記録
- [PHASE2_COMPLETION_REPORT.md](PHASE2_COMPLETION_REPORT.md) - Phase 2詳細レポート
- [CPP_DEPENDENCIES.md](CPP_DEPENDENCIES.md) - 依存関係マップ
- [CPP_FILE_FUNCTION_INDEX.md](CPP_FILE_FUNCTION_INDEX.md) - ファイル索引
- [Lib/README.md](../Lib/README.md) - Libフォルダ構造説明

---

**次のアクション**: Phase 3 Task 3.1 実行開始  
**担当**: AI Agent  
**期限**: 2026年1月4日中

