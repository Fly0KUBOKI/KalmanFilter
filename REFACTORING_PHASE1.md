# Phase 1: ドキュメント整理

**目標**: 不要なMarkdownを削除し、必要な情報を統合  
**所要時間**: 30分  
**リスク**: 低

---

## 1. 削除対象ファイル一覧

### ルートディレクトリ

| ファイル | 行数 | 削除理由 |
|---------|------|---------|
| `PHASE3_PROGRESS.md` | 374 | 過去のPhase 3進捗。完了済み、履歴として不要 |
| `PHASE3_CURRENT_REFACTORING.md` | 158 | 完了済みタスク記録。現在の状況と混乱を招く |
| `PROJECT_STATUS.md` | 425 | 過去のステータス。新計画に置換 |
| `memo.md` | 50 | 一時メモ。整理後削除 |

**合計削除行数**: 約1,007行

### docs/ ディレクトリ

| ファイル | 行数 | 削除理由 |
|---------|------|---------|
| `BINARY_ANALYSIS_SUMMARY.md` | - | 過去のバイナリ分析、不要 |
| `BINARY_MANAGEMENT.md` | - | 過去の管理情報、不要 |
| `BUILD_OPTIMIZATION_RECOMMENDATIONS.md` | - | 過去の推奨事項、不要 |
| `COMPILER_ANALYSIS_FINAL.md` | - | 過去のコンパイラ分析、不要 |
| `COMPILER_DEPENDENCY_ROOT_CAUSE.md` | - | 過去の原因分析、不要 |
| `COMPILER_FIX_PLAN.md` | - | 過去の修正計画、完了済み |
| `COMPILER_ISSUE_ROOT_CAUSE_FINAL.md` | - | 過去の原因分析、不要 |
| `ENVIRONMENT_DEPENDENCY_GUIDE.md` | - | 過去のガイド、不要 |
| `ENVIRONMENT_QA.md` | - | 過去のQA、不要 |
| `ENVIRONMENT_QUICK_START.md` | - | 過去のガイド、不要 |
| `ENVIRONMENT_SETUP.md` | - | 過去のセットアップ、不要 |
| `IMPLEMENTATION_CHECKLIST.md` | - | 過去のチェックリスト、不要 |
| `IMPLEMENTATION_SUMMARY.md` | - | 過去のサマリー、不要 |
| `PHASE1_TYPE_UNIFICATION_PLAN.md` | - | 完了済み計画、不要 |

**削除ファイル数**: 14ファイル

---

## 2. 保持・更新するファイル

### 保持するファイル

| ファイル | アクション | 備考 |
|---------|----------|------|
| `.github/copilot-instructions.md` | **更新必要** | 新構造を反映、削除されたファイル参照を削除 |
| `docs/README.md` | **更新必要** | 現在のアーキテクチャを記載 |
| `docs/CODING_STANDARDS.md` | **更新必要** | 新規約を反映 |
| `docs/CPP_ARCHITECTURE.md` | **更新必要** | 新構造を反映 |
| `docs/CPP_INPUT_OUTPUT_SPEC.md` | **保持** | 型仕様は有効 |
| `docs/LIB_STRUCTURE.md` | **更新必要** | 新構造を反映 |

### 新規作成ファイル

| ファイル | 目的 |
|---------|------|
| `REFACTORING_PLAN.md` | 包括的リファクタリング計画（作成済み） |
| `REFACTORING_PHASE1.md` | 本ファイル |
| `REFACTORING_PHASE2.md` | 環境依存・デバッグ廃止の詳細 |
| `REFACTORING_PHASE3.md` | 冗長コード廃止の詳細 |
| `REFACTORING_PHASE4.md` | クラス設計再構築の詳細 |
| `REFACTORING_PHASE5.md` | ファイル構成再構築の詳細 |
| `REFACTORING_PHASE6.md` | コメント・規約統一の詳細 |

---

## 3. 実施手順

### Step 1: バックアップ作成（任意）

```bash
cd c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter

# バックアップディレクトリ作成
mkdir -p .archive\docs_backup_20260111

# ルートのmdファイルをバックアップ
copy PHASE3_PROGRESS.md .archive\docs_backup_20260111\
copy PHASE3_CURRENT_REFACTORING.md .archive\docs_backup_20260111\
copy PROJECT_STATUS.md .archive\docs_backup_20260111\
copy memo.md .archive\docs_backup_20260111\
```

### Step 2: ルートディレクトリのファイル削除

```bash
# 削除実行
del PHASE3_PROGRESS.md
del PHASE3_CURRENT_REFACTORING.md
del PROJECT_STATUS.md
del memo.md
```

### Step 3: docs/ ディレクトリの整理

```bash
cd docs

# 不要ファイル削除
del BINARY_ANALYSIS_SUMMARY.md
del BINARY_MANAGEMENT.md
del BUILD_OPTIMIZATION_RECOMMENDATIONS.md
del COMPILER_ANALYSIS_FINAL.md
del COMPILER_DEPENDENCY_ROOT_CAUSE.md
del COMPILER_FIX_PLAN.md
del COMPILER_ISSUE_ROOT_CAUSE_FINAL.md
del ENVIRONMENT_DEPENDENCY_GUIDE.md
del ENVIRONMENT_QA.md
del ENVIRONMENT_QUICK_START.md
del ENVIRONMENT_SETUP.md
del IMPLEMENTATION_CHECKLIST.md
del IMPLEMENTATION_SUMMARY.md
del PHASE1_TYPE_UNIFICATION_PLAN.md

# 残ったファイル確認
dir *.md
```

**期待される残存ファイル**:
- `README.md`
- `CODING_STANDARDS.md`
- `CPP_ARCHITECTURE.md`
- `CPP_INPUT_OUTPUT_SPEC.md`
- `LIB_STRUCTURE.md`

### Step 4: copilot-instructions.md の更新

`.github/copilot-instructions.md` から以下を更新:

1. **削除されたファイルへの参照を削除**:
   - `PLAN.md` への参照削除
   - `ROADMAP_TO_PHASE_13.md` への参照削除
   - `TYPE_MIX_REPORT.md` への参照削除
   - `FILE_DUPLICATION_REPORT.md` への参照削除

2. **ファイルパスの更新**:
   - `kalman/cpp/markdown/CPP_INPUT_OUTPUT_SPEC.md` → `docs/CPP_INPUT_OUTPUT_SPEC.md`

3. **重要ファイルテーブルの更新**

---

## 4. 更新すべき箇所の詳細

### `.github/copilot-instructions.md` の修正箇所

#### 修正1: 参考資料セクション（末尾付近）

```markdown
## 【参考資料】
- PLAN.md — 進捗・成功指標  ← 削除
- ROADMAP_TO_PHASE_13.md — 全体ロードマップ  ← 削除
- kalman/cpp/FILE_DUPLICATION_REPORT.md — ファイル重複解析  ← 削除
- kalman/cpp/Lib/README.md — ライブラリモジュール説明
```

変更後:
```markdown
## 【参考資料】
- docs/CPP_ARCHITECTURE.md — C++アーキテクチャ説明
- docs/CPP_INPUT_OUTPUT_SPEC.md — 型マッピング・配列レイアウト
- docs/CODING_STANDARDS.md — コーディング規約
```

#### 修正2: 型マッピング参照

```markdown
- [CPP_INPUT_OUTPUT_SPEC.md](kalman/cpp/markdown/CPP_INPUT_OUTPUT_SPEC.md)で型マッピング確認必須
```

変更後:
```markdown
- [CPP_INPUT_OUTPUT_SPEC.md](docs/CPP_INPUT_OUTPUT_SPEC.md)で型マッピング確認必須
```

---

## 5. 完了確認チェックリスト

- [ ] `PHASE3_PROGRESS.md` 削除
- [ ] `PHASE3_CURRENT_REFACTORING.md` 削除
- [ ] `PROJECT_STATUS.md` 削除
- [ ] `memo.md` 削除
- [ ] `docs/BINARY_*.md` 削除
- [ ] `docs/COMPILER_*.md` 削除
- [ ] `docs/ENVIRONMENT_*.md` 削除
- [ ] `docs/IMPLEMENTATION_*.md` 削除
- [ ] `docs/PHASE1_*.md` 削除
- [ ] `.github/copilot-instructions.md` 更新
- [ ] 残存ファイル確認（5ファイル + README）

---

## 6. Phase 1 完了後の状態

### ファイル構成

```
KalmanFilter/
├── .github/
│   └── copilot-instructions.md  # 更新済み
├── docs/
│   ├── README.md
│   ├── CODING_STANDARDS.md
│   ├── CPP_ARCHITECTURE.md
│   ├── CPP_INPUT_OUTPUT_SPEC.md
│   └── LIB_STRUCTURE.md
├── REFACTORING_PLAN.md          # 新規
├── REFACTORING_PHASE1.md        # 新規（本ファイル）
├── REFACTORING_PHASE2.md        # 新規
├── REFACTORING_PHASE3.md        # 新規
├── REFACTORING_PHASE4.md        # 新規
├── REFACTORING_PHASE5.md        # 新規
├── REFACTORING_PHASE6.md        # 新規
└── kalman/
    └── ...
```

### 削減効果

| 項目 | Before | After | 削減率 |
|-----|--------|-------|-------|
| Markdownファイル数 | 22 | 12 | 45% |
| 推定総行数 | 3,000+ | 1,500 | 50% |

---

## 7. 次のPhaseへの移行条件

- [x] 全削除対象ファイルが削除済み
- [x] 残存ファイルが正しく保持されている
- [x] copilot-instructions.md が更新済み
- [x] Git commit 完了

**次のPhase**: Phase 2 - 環境依存・デバッグ機能の廃止
