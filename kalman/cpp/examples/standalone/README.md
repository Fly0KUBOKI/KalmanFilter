# Kalman Filter Standalone Examples

このフォルダには、Kalman FilterライブラリをMATLAB MEX以外の環境で使用するサンプルコードが含まれています。

---

## 📁 サンプル一覧

### 1. `main_simple.cpp` — 最小限の使用例

**説明**: 
- 10ステップのフィルタ更新を実行
- 静止状態のIMUデータをシミュレート
- 推定結果を標準出力に表示

**ビルド方法** (Linux/Mac/MinGW):
```bash
cd kalman/cpp/examples/standalone

g++ -std=c++11 -I../../Lib \
    ../../Lib/Common/src/standalone.cpp \
    ../../Lib/ESKF/src/*.cpp \
    ../../Lib/MEUKF/src/*.cpp \
    ../../Lib/Matrix/src/*.cpp \
    main_simple.cpp -o kalman_simple

./kalman_simple
```

**ビルド方法** (Windows MSVC):
```cmd
cl /EHsc /std:c++11 /I..\..\Lib ^
   ..\..\Lib\Common\src\standalone.cpp ^
   ..\..\Lib\ESKF\src\*.cpp ^
   ..\..\Lib\MEUKF\src\*.cpp ^
   ..\..\Lib\Matrix\src\*.cpp ^
   main_simple.cpp /Fe:kalman_simple.exe

kalman_simple.exe
```

**実行結果例**:
```
=== Kalman Filter Standalone Example ===

1. Setting filter type to ESKF...
2. Initializing filter...
   Filter initialized successfully.

3. Preparing sensor data...
   ...

=== Estimation Results ===

Position [m] (ENU frame):
  East:      0.123
  North:     0.456
  Up:        0.789

...
```

---

## 🚀 今後のサンプル（計画中）

### `main_from_csv.cpp` — CSVファイルから読込

**計画内容**:
- センサーデータをCSVファイルから読み込み
- フィルタ更新をループ実行
- 推定結果をCSVに保存
- MATLABの`run_simulation.m`と同等の機能

**使用方法（予定）**:
```bash
./kalman_csv --input sensor_data.csv --output estimation.csv
```

### `main_realtime.cpp` — リアルタイムストリーミング

**計画内容**:
- UDPソケットからセンサーデータ受信
- リアルタイムフィルタ更新
- 結果をUDPで送信
- ログファイルに記録

---

## 🔧 ビルドのトラブルシューティング

### エラー: "undefined reference to ..."

**原因**: ソースファイル（.cpp）のリンク漏れ

**解決方法**: 必要なすべての.cppファイルをコンパイルコマンドに含める
```bash
# NG: standalone.cppを含めていない
g++ main_simple.cpp -o kalman_simple

# OK: 必要なすべての.cppを含める
g++ ../../Lib/Common/src/standalone.cpp \
    ../../Lib/ESKF/src/*.cpp \
    main_simple.cpp -o kalman_simple
```

### エラー: "no such file or directory: standalone.hpp"

**原因**: インクルードパスが正しくない

**解決方法**: `-I../../Lib` を追加
```bash
g++ -I../../Lib main_simple.cpp -o kalman_simple
```

### Windows環境でのビルドエラー

**原因**: パス区切り文字の違い、ワイルドカードの展開

**解決方法**: 
1. CMakeを使用（推奨、今後提供予定）
2. 個別にファイル名を指定
3. PowerShellのワイルドカード機能を使用

```powershell
# PowerShellでのビルド例
$sources = Get-ChildItem -Recurse ..\..\Lib\*.cpp | Select-Object -ExpandProperty FullName
g++ -std=c++11 -I..\..\Lib $sources main_simple.cpp -o kalman_simple.exe
```

---

## 📖 関連ドキュメント

- [Lib/README.md](../../Lib/README.md) — ライブラリAPI詳細
- [docs/STANDALONE_API_REFACTORING_PLAN.md](../../../../docs/STANDALONE_API_REFACTORING_PLAN.md) — API改善計画
- [docs/CPP_ARCHITECTURE.md](../../../../docs/CPP_ARCHITECTURE.md) — アーキテクチャ設計

---

## 🤝 貢献

新しいサンプルコードの追加、既存サンプルの改善は歓迎です。

**提案されているサンプル**:
- [ ] CSV読込サンプル
- [ ] リアルタイムストリーミング
- [ ] 複数フィルタ並列実行（API改善後）
- [ ] カスタムセンサー追加例
- [ ] パラメータチューニング例

---

**更新履歴**:
- 2026-01-14: 初版作成、main_simple.cpp追加
