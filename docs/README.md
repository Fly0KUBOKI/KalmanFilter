# KalmanFilter プロジェクトドキュメント

## 📚 ドキュメント構成（2026年1月11日更新）

このプロジェクトは **MATLAB実験フロントエンド + C++ MEX高速計算** のハイブリッド実装による高精度慣性航法システムです。

### ✅ **重要: コンパイラ依存問題を完全解決**

MinGW/MSVCともに正常動作を確認（2026-01-11）。**真の原因は型の不整合でした。**

- **[COMPILER_ISSUE_ROOT_CAUSE_FINAL.md](COMPILER_ISSUE_ROOT_CAUSE_FINAL.md)** — 真の根本原因と修正履歴（必読）
- **[CODING_STANDARDS.md](CODING_STANDARDS.md)** — 再発防止のためのコーディング規約
- **[BUILD_OPTIMIZATION_RECOMMENDATIONS.md](BUILD_OPTIMIZATION_RECOMMENDATIONS.md)** — ビルド最適化の推奨事項

### 🚀 クイックスタート

1. **[PROJECT_STATUS.md](../PROJECT_STATUS.md)** — プロジェクト全体ロードマップ・進行状況・メトリクス
2. **[.github/copilot-instructions.md](../.github/copilot-instructions.md)** — AI開発者向けガイド・コード規約
3. **標準テスト**: `run_batch_10sets()` — 10セット実行で推定精度を検証

### 📖 詳細リファレンス

#### システムアーキテクチャ理解
- **[LIB_STRUCTURE.md](LIB_STRUCTURE.md)** — Lib層7層構造・全関数リスト・モジュール依存関係
- **[CPP_ARCHITECTURE.md](CPP_ARCHITECTURE.md)** — C++アーキテクチャ・MEX層設計・最適化技法
- **[CPP_INPUT_OUTPUT_SPEC.md](CPP_INPUT_OUTPUT_SPEC.md)** — 型マッピング・I/O仕様・MEX-MATLAB間の変換

#### ビルド・環境・コンパイラ
- **[COMPILER_ISSUE_ROOT_CAUSE_FINAL.md](COMPILER_ISSUE_ROOT_CAUSE_FINAL.md)** — 型の不整合問題の真の原因と修正（重要）
- **[CODING_STANDARDS.md](CODING_STANDARDS.md)** — 型の使用規則・再発防止策（必読）
- **[BUILD_OPTIMIZATION_RECOMMENDATIONS.md](BUILD_OPTIMIZATION_RECOMMENDATIONS.md)** — デバッグビルド・最適化の推奨事項
- **[ENVIRONMENT_QUICK_START.md](ENVIRONMENT_QUICK_START.md)** — 環境セットアップ（コンパイラ選択、ビルド手順）

#### 廃止（統合完了または誤った仮説）
- ~~COMPILER_ANALYSIS_FINAL.md~~ → 誤った分析のため削除（真の原因は型の不整合）
- ~~COMPILER_DEPENDENCY_ROOT_CAUSE.md~~ → 誤った仮説のため削除
- ~~COMPILER_FIX_PLAN.md~~ → 不要な修正計画のため削除
- ~~diagnose_compiler_difference.m~~ → 不正確なテストツールのため削除
- ~~CODE_PORTABILITY_ANALYSIS.md~~ → 統合済み
- ~~TROUBLESHOOTING_REFERENCE.md~~ → PROJECT_STATUS.md に統合
- ~~PROJECT_OVERVIEW.md~~ → PROJECT_STATUS.md に統合

### 🎯 現在の達成状況（2026-01-11 更新）

| 指標 | 達成値 | 目標値 | 状況 |
|------|--------|--------|------|
| **Position RMSE** | **0.32m** | < 2.0m | ✅ **達成** (平均値、10セット) |
| **Attitude RMSE** | **0.31/0.31/0.79°** | < 1.0° | ✅ **達成** (Roll/Pitch/Yaw) |
| **成功率** | **100% (10/10)** | > 90% | ✅ **達成** |
| **実行時間** | < 25秒/Run | - | ✅ **高速** |
| **コンパイラ互換性** | MinGW/MSVC両対応 | - | ✅ **達成** |

### ⚡ 使用例

```matlab
% 基本的な使用フロー
cd kalman

% MEXビルド (初回のみ)
cd cpp/build
build_mex();
clear mex
cd ../..

% シミュレーション実行
run_simulation(42, true);     % 単体テスト
run_batch_10sets();           % 統計テスト (10seeds)

% 結果確認
type('Results/estimation_01.csv')
```

### 📂 プロジェクト構造

```
KalmanFilter/
├── docs/                    # 📖 このドキュメント群
├── .github/                 # 🤖 GitHub設定・AI指示
├── PLAN.md                  # 📋 進捗レポート
└── kalman/                  # 🏠 メインプロジェクト
    ├── *.m                  # MATLAB実行スクリプト
    ├── GenerateData/        # 📊 センサーデータ生成
    ├── Graph/               # 📈 可視化ツール
    ├── FFT/                # 🔄 周波数解析
    ├── Results/            # 📄 実行結果・ログ
    └── cpp/                # ⚡ C++ MEX実装
        ├── build/          # 🔨 ビルドシステム
        ├── bin/            # 📦 MEXバイナリ
        ├── MEX/            # 🔌 MATLABインターフェース
        └── Lib/            # 📚 ライブラリ実装
```

### 🔧 開発ワークフロー

1. **変更**: C++コード修正
2. **ビルド**: `build_mex()` — 30秒（自動的に `clear mex` 実行）
3. **テスト**: `run_batch_10sets()` — 25秒（推奨）  
4. **検証**: Results/batch_10sets_summary.csv で成功率・RMSE確認
5. **診断**: Results/log/ 内のタイムスタンプ付きログで詳細確認

**標準テスト**:
```matlab
cd kalman
run_batch_10sets();  % 10セット実行、100%成功が期待値
```

### 🎓 学習リソース

- **初心者**: [ENVIRONMENT_QUICK_START.md](ENVIRONMENT_QUICK_START.md) → [LIB_STRUCTURE.md](LIB_STRUCTURE.md)
- **MATLAB開発者**: [CPP_INPUT_OUTPUT_SPEC.md](CPP_INPUT_OUTPUT_SPEC.md)
- **C++開発者**: [CPP_ARCHITECTURE.md](CPP_ARCHITECTURE.md) → [LIB_STRUCTURE.md](LIB_STRUCTURE.md)
- **ビルド最適化**: [BUILD_OPTIMIZATION_RECOMMENDATIONS.md](BUILD_OPTIMIZATION_RECOMMENDATIONS.md)
- **AI開発者**: [../.github/copilot-instructions.md](../.github/copilot-instructions.md)

---

## 🤝 貢献・サポート

- **Issues**: バグ報告・機能要求
- **Pull Requests**: コード改善・ドキュメント修正  
- **Discussions**: 技術議論・質問

このドキュメントは2026年1月4日時点での情報です。最新の状況は [PLAN.md](../PLAN.md) をご確認ください。