# Kalman Filter Examples

このフォルダには、Kalman Filterライブラリをさまざまなプラットフォームやユースケースで使用するサンプルコードが含まれています。

---

## 📂 フォルダ構成

```
examples/
├── standalone/          # スタンドアロンC++アプリケーション
│   ├── main_simple.cpp      # 最小限の使用例（実装済み）
│   ├── main_from_csv.cpp    # CSVデータ処理（計画中）
│   └── README.md            # ビルド方法・使用例
│
├── embedded/            # 組み込みシステム向けサンプル（計画中）
│   ├── stm32_freertos/      # STM32 + FreeRTOS
│   ├── esp32/               # ESP32
│   └── README.md
│
└── README.md            # このファイル
```

---

## 🚀 クイックスタート

### 最小限のサンプル実行（5分）

```bash
# 1. サンプルフォルダに移動
cd kalman/cpp/examples/standalone

# 2. ビルド（Linux/Mac/MinGW）
g++ -std=c++11 -I../../Lib \
    ../../Lib/Common/src/standalone.cpp \
    ../../Lib/ESKF/src/*.cpp \
    ../../Lib/MEUKF/src/*.cpp \
    ../../Lib/Matrix/src/*.cpp \
    main_simple.cpp -o kalman_simple

# 3. 実行
./kalman_simple
```

**期待される出力**:
```
=== Kalman Filter Standalone Example ===

1. Setting filter type to ESKF...
2. Initializing filter...
   Filter initialized successfully.

...

=== Estimation Results ===

Position [m] (ENU frame):
  East:      0.000
  North:     0.000
  Up:        0.000

...
```

---

## 📚 サンプル詳細

### 実装済み

#### [standalone/main_simple.cpp](standalone/main_simple.cpp)
- **難易度**: ⭐ 初級
- **行数**: ~150行
- **説明**: 静止状態のIMUデータで10ステップ更新
- **学習内容**: API基本、初期化、更新、状態取得

---

### 計画中（Phase 2以降）

#### `standalone/main_from_csv.cpp`
- **難易度**: ⭐⭐ 中級
- **説明**: CSVファイルからセンサーデータを読み込み、フィルタ更新、結果をCSV保存
- **学習内容**: ファイルI/O、ループ処理、エラーハンドリング

#### `embedded/stm32_freertos/`
- **難易度**: ⭐⭐⭐ 上級
- **説明**: STM32マイコン上でFreeRTOSタスクとして動作
- **学習内容**: リアルタイムOS、割り込み処理、メモリ最適化

#### `embedded/esp32/`
- **難易度**: ⭐⭐ 中級
- **説明**: ESP32上でWi-Fi経由でセンサーデータ受信
- **学習内容**: Wi-Fiスタック、ESP-IDF、RTOS

---

## 🛠️ ビルド環境

### 推奨環境

| 環境 | コンパイラ | 標準 | 備考 |
|------|----------|------|------|
| **Linux** | GCC 7+ | C++11 | Ubuntu 18.04以降 |
| **macOS** | Clang 10+ | C++11 | Xcode Command Line Tools |
| **Windows** | MinGW-w64 | C++11 | MATLAB同梱版推奨 |
| **Windows** | MSVC 2017+ | C++11 | Visual Studio 2017以降 |

### 依存ライブラリ

**必須**:
- なし（標準C++ライブラリのみ）

**オプション（サンプルによる）**:
- CSVサンプル: なし（独自パーサー使用）
- 組み込みサンプル: CMSIS、HAL、FreeRTOSなど

---

## 📖 学習パス

### 初心者向け
1. [standalone/main_simple.cpp](standalone/main_simple.cpp) を読む
2. ビルド・実行して結果を確認
3. センサーデータを変更して挙動を観察
4. [Lib/README.md](../Lib/README.md) でAPI詳細を学習

### 中級者向け
1. CSVサンプル実装（計画中のものを自作）
2. パラメータチューニング
3. カスタムセンサー追加
4. [docs/CPP_ARCHITECTURE.md](../../../docs/CPP_ARCHITECTURE.md) でアーキテクチャ理解

### 上級者向け
1. 組み込み環境への移植
2. リアルタイム制約の実装
3. メモリ最適化（動的メモリ削除）
4. [STANDALONE_API_REFACTORING_PLAN.md](../../../docs/STANDALONE_API_REFACTORING_PLAN.md) に貢献

---

## 🤝 貢献

新しいサンプルの追加、既存サンプルの改善を歓迎します。

### サンプル作成ガイドライン
1. **README.md**: ビルド方法と実行結果を明記
2. **コメント**: 初心者にも理解できる詳細なコメント
3. **エラーハンドリング**: すべての関数の戻り値をチェック
4. **出力**: 処理の各ステップを標準出力で確認可能に

### 提案されているサンプル
- [ ] Python binding（pybind11使用）
- [ ] ROS 2ノード
- [ ] WebAssembly（ブラウザ実行）
- [ ] Unity plugin（ゲームエンジン連携）

---

## 🔗 関連リンク

- [プロジェクトREADME](../../../docs/README.md)
- [ライブラリAPI](../Lib/README.md)
- [API改善計画](../../../docs/STANDALONE_API_REFACTORING_PLAN.md)
- [コーディング規約](../../../docs/CODING_STANDARDS.md)

---

**更新履歴**:
- 2026-01-14: 初版作成、standalone/main_simple.cpp実装
