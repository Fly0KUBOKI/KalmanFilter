# プロジェクト概要 — KalmanFilter

## 🎯 概要

KalmanFilterは**MATLAB実験フロントエンド + C++ MEX高速計算**のハイブリッド実装による高精度慣性航法システムです。

### 設計思想
- **実験の柔軟性**: MATLAB による直感的なデータ生成・可視化・パラメータ調整
- **計算の高速性**: C++ MEX による15×15共分散更新・四元数演算の最適化
- **開発効率**: 自動ビルドシステム・10seed統計テスト・詳細ログ管理

## 📊 現在の性能指標

| 指標 | 達成値 | 備考 |
|------|--------|------|
| **Position RMSE** | 0.80-0.91m | GPS/IMU統合による高精度測位 |
| **Attitude RMSE** | 0.25-0.30° | 四元数ベース姿勢推定 |
| **成功率** | 100% (10/10 seeds) | 統計的安定性確保 |
| **計算時間** | < 1分 (50秒シミュレーション) | MEX最適化効果 |

## 🏗️ システム構成

```
┌─ MATLAB Frontend ─────────────────────┐    ┌─ C++ MEX Engine ──────────────────┐
│                                       │    │                                   │
│ • データ生成 (GenerateData/)          │    │ • ESKF Core (状態予測・更新)      │
│ • 実行制御 (run_simulation.m)         │◄──►│ • センサー統合・外れ値検出         │
│ • バッチテスト (run_batch_10sets.m)   │    │ • 四元数・行列演算 (最適化済み)    │
│ • 可視化・分析 (Graph/, FFT/)         │    │ • 共分散管理・数値安定性          │
│                                       │    │                                   │
└───────────────────────────────────────┘    └───────────────────────────────────┘
                     │
                     ▼
         ┌─ 出力・検証システム ──────┐
         │ • Results/estimation_*.csv │
         │ • 10seed統計解析           │
         │ • ログ・デバッグ情報       │
         └───────────────────────────┘
```

## 🔧 技術仕様

### 状態ベクトル (15次元)
```
[p(3), v(3), q(4), ba(3), bg(3)]
```
- **p**: 位置 [m]
- **v**: 速度 [m/s] 
- **q**: 四元数 [w,x,y,z] (スカラー先頭)
- **ba**: 加速度バイアス [m/s²]
- **bg**: ジャイロバイアス [rad/s]

### センサー統合
- **IMU**: 加速度・ジャイロ・磁気センサー
- **GPS**: 緯度・経度・高度 (double精度)
- **気圧計**: 高度補正
- **外れ値検出**: ロバスト統計・マハラノビス距離

### 型システム
- **センサーデータ**: `float32` 統一 (GPS除く)
- **GPS座標**: `double` 高精度
- **共分散行列**: `float32[15×15]` column-major
- **MEX同期**: `clear mex` 必須・対称化処理

## 🛠️ 開発環境要件

### 必須ソフトウェア
- **MATLAB R2020b+**: MEX対応・Signal Processing Toolbox
- **Microsoft Visual C++ 2022**: MEXコンパイラ
- **Git**: バージョン管理

### 推奨環境
- **Windows 10/11**: 主開発プラットフォーム
- **RAM 8GB+**: 大規模シミュレーション用
- **SSD**: ビルド・テスト高速化

## 📁 ディレクトリ構造

```
KalmanFilter/
├── .github/copilot-instructions.md      # AI開発指針
├── PLAN.md                              # 進捗・成果サマリー
├── docs/                                # 新しいドキュメント体系
│   ├── PROJECT_OVERVIEW.md              # (このファイル)
│   ├── MATLAB_COMPONENTS.md             
│   ├── CPP_ARCHITECTURE.md              
│   └── BUILD_AND_WORKFLOW.md            
└── kalman/
    ├── run_simulation.m                 # 単体テスト実行
    ├── run_batch_10sets.m               # 10seed統計テスト
    ├── GenerateData/                    # センサーデータ生成
    ├── Graph/                           # 可視化ツール
    ├── FFT/                            # 周波数解析
    ├── Results/                         # 出力・ログ
    └── cpp/                            # C++実装
        ├── build/build_mex.m            # 自動ビルドシステム
        ├── bin/                         # MEXバイナリ
        ├── MEX/                         # MEXインターフェース
        └── Lib/                         # ライブラリ実装
```

## 🚀 クイックスタート

```matlab
% 1. プロジェクトディレクトリに移動
cd kalman

% 2. MEXライブラリをビルド
cd cpp/build
build_mex();
clear mex
cd ../..

% 3. 単体テスト実行
run_simulation(42, true);

% 4. 結果確認
type('Results/estimation_01.csv')
```

## 🎯 今後の展開

### Phase 13 目標
- Innovation計算の完全実装
- スタンドアロンC++版の提供
- CMakeベースビルドシステム
- 追加センサー対応 (カメラ・LiDAR)

## 📞 開発支援

### 重要な参照先
- [MATLAB Components](MATLAB_COMPONENTS.md) — MATLABコンポーネント詳細
- [C++ Architecture](CPP_ARCHITECTURE.md) — C++実装構造
- [Build Workflow](BUILD_AND_WORKFLOW.md) — ビルド・テスト手順
- [GitHub Copilot Instructions](../.github/copilot-instructions.md) — AI開発ガイド

### トラブルシューティング
- **ビルドエラー**: `clear mex` → `build_mex()` 再実行
- **数値差異**: 型混在・四元数順序・共分散対称性を確認  
- **性能低下**: `Results/log/` の詳細ログ確認