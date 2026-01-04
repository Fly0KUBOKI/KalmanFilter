# KalmanFilter プロジェクトドキュメント

## 📚 ドキュメント構成

このプロジェクトは **MATLAB実験フロントエンド + C++ MEX高速計算** のハイブリッド実装による高精度慣性航法システムです。

### 🚀 クイックスタート

1. **[プロジェクト概要](PROJECT_OVERVIEW.md)** — 全体像・性能指標・システム構成
2. **[ビルド・実行手順](BUILD_AND_WORKFLOW.md)** — 5分で始める開発サイクル
3. **[GitHub Copilot 指示](../.github/copilot-instructions.md)** — AI開発者向けガイド

### 📖 詳細ドキュメント

#### システム理解
- **[MATLABコンポーネント](MATLAB_COMPONENTS.md)** — データ生成・実行制御・可視化・テストフレームワーク
- **[C++アーキテクチャ](CPP_ARCHITECTURE.md)** — MEX層・ESKFライブラリ・最適化技法・数値安定性

#### 運用・保守
- **[トラブルシューティング](TROUBLESHOOTING_REFERENCE.md)** — 既知問題・デバッグ技法・品質管理・MEX API仕様

### 🎯 現在の達成状況

| 指標 | 達成値 | 目標値 | 状況 |
|------|--------|--------|------|
| **Position RMSE** | 0.80-0.91m | < 2.0m | ✅ **達成** |
| **Attitude RMSE** | 0.25-0.30° | < 1.0° | ✅ **達成** |
| **成功率** | 100% (10/10) | > 90% | ✅ **達成** |
| **実行時間** | < 60秒 | - | ✅ **高速** |

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
2. **ビルド**: `build_mex({'target'})` — 30秒
3. **テスト**: `run_simulation(42, true)` — 60秒  
4. **検証**: `run_batch_10sets()` — 10分
5. **診断**: ログ・CSV結果確認

### 🎓 学習リソース

- **初心者**: [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md) → [BUILD_AND_WORKFLOW.md](BUILD_AND_WORKFLOW.md)
- **MATLAB開発者**: [MATLAB_COMPONENTS.md](MATLAB_COMPONENTS.md)
- **C++開発者**: [CPP_ARCHITECTURE.md](CPP_ARCHITECTURE.md) 
- **トラブル時**: [TROUBLESHOOTING_REFERENCE.md](TROUBLESHOOTING_REFERENCE.md)
- **AI開発者**: [../.github/copilot-instructions.md](../.github/copilot-instructions.md)

---

## 🤝 貢献・サポート

- **Issues**: バグ報告・機能要求
- **Pull Requests**: コード改善・ドキュメント修正  
- **Discussions**: 技術議論・質問

このドキュメントは2026年1月4日時点での情報です。最新の状況は [PLAN.md](../PLAN.md) をご確認ください。