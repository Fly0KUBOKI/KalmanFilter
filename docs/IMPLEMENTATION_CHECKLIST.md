# Implementation Checklist: MEX Binary Management

**目的**: PCごとのMEXバイナリ差異を管理し、Git リポジトリを整理する  
**実施期間**: 即座（短期）、1週間（中期）、1ヶ月（長期）  
**対象**: KalmanFilter プロジェクト開発チーム

---

## ✅ **短期チェックリスト（即座に実施）** — 1日以内

### 1. `.gitignore` の確認と更新

**状態**: ✅ 既に `.mexw64` は含まれている  
**確認**:
```bash
grep "*.mexw64" .gitignore  # 出力: kalman/cpp/bin/*.mexw64
```

**追加検討**（Linux対応の場合）:
```
# kalman/cpp/bin ディレクトリ全体の生成物を無視
kalman/cpp/bin/*.mexw64
kalman/cpp/bin/*.mexa64
kalman/cpp/build/*.pdb
kalman/cpp/build/build_env_report_*.txt
```

**実施**:
```bash
cd <repo_root>
git add .gitignore
git commit -m "docs: clarify binary file exclusion in .gitignore"
```

### 2. README.md に初期化手順を追加

**場所**: プロジェクトルート `README.md` の "セットアップ" セクション

**追加内容**:
```markdown
## セットアップ

### 前提環境
- MATLAB R2024a 以上
- Windows 10/11 + Visual Studio 2022 or BuildTools
- (または Linux/macOS + GCC/Clang)

### 初回セットアップ（必須）

```matlab
% 1. リポジトリをクローン
git clone https://github.com/your-org/KalmanFilter.git
cd KalmanFilter

% 2. MEX バイナリをビルド（必須）
% このステップで、環境に最適化された mex_run_eskf.mexw64 を生成します
cd kalman/cpp/build
build_mex();  % または build_mex_verbose(); で詳細ログを出力

% 3. 動作確認
cd ../..
run_simulation(1, true);  % 最初の1シード で検証
```

### 注意事項
- **重要**: 初回 clone 後は、必ず `build_mex()` を実行してください
- MEX バイナリはリポジトリに含まれません（各環境で生成）
- 問題が発生した場合は、`diagnose_environment()` を実行して環境を診断してください
```

**実施**:
```bash
# README.md を編集
# 上記内容を "セットアップ" セクションに追加
git add README.md
git commit -m "docs: add MEX binary setup instructions"
```

### 3. 既存の .mexw64 ファイルをリポジトリから削除（済み確認）

**状態確認**:
```bash
git status kalman/cpp/bin/
# → "nothing to commit" が理想的
```

**もしファイルが Git で追跡されている場合**:
```bash
git rm --cached kalman/cpp/bin/*.mexw64
git commit -m "chore: remove .mexw64 binaries from version control"
```

### ✅ **短期チェックリスト 確認用コマンド**

```matlab
% MATLAB コマンドラインで実行
cd kalman
diagnose_environment();  % 環境診断
% 出力: environment_diagnostic.txt が Results/ に保存される

% その他
which mex_run_eskf  % MEX が見つかるか確認
mex_run_eskf('help')  % 機能確認
```

---

## 🔄 **中期チェックリスト（1週間以内）** — 複数エンジニア対応

### 1. build_mex_verbose.m の動作確認と配置

**ファイル**: `kalman/cpp/build/build_mex_verbose.m` ✅ (作成済み)

**テスト実行**:
```matlab
cd kalman/cpp/build
build_mex_verbose();  % 全MEXをビルド + 環境情報ログ出力
```

**期待出力**:
```
【1】MATLAB Information
  MATLAB Version: 25.2
  Release: R2025b
  MEX Extension: mexw64

【3】C++ Compiler Information
  MSVC Version: ...

【7】Post-Build Binary Analysis
  mex_run_eskf.mexw64:
    Size: 155,000 bytes
    MD5: 15269c6a49f9c945b31c9c824eecd0b2
    ✓ No debug symbols detected
```

**保存**: 上記スクリプトは既に `kalman/cpp/build/build_mex_verbose.m` に配置済み

### 2. BINARY_MANAGEMENT.md ドキュメント配置確認

**ファイル**: `docs/BINARY_MANAGEMENT.md` ✅ (作成済み)

**内容確認**:
```bash
cat docs/BINARY_MANAGEMENT.md | head -50
# 確認項目:
# ✓ バイナリサイズ差の原因（5つの可能性）
# ✓ Git 戦略（4つのオプション）
# ✓ トラブルシューティング Q&A
# ✓ 実装例コード
```

**配置**:
```bash
git add docs/BINARY_MANAGEMENT.md
git commit -m "docs: add binary management guide"
```

### 3. diagnose_environment.m の拡張（バイナリメタデータ追加）

**現在の状態**: ✅ 5つのテストが実装済み

**拡張内容** (オプション):
```matlab
% diagnose_environment.m に TEST 6 を追加
% 【TEST 6】MEX バイナリメタデータ
fprintf('\n【TEST 6】MEX Binary Metadata\n');
fprintf('----------\n');

bin_dir = fullfile(pwd, 'cpp', 'bin');
mex_files = dir(fullfile(bin_dir, ['*.' mexext]));

if isempty(mex_files)
    fprintf('❌ MEX バイナリが見つかりません\n');
    fprintf('   → cd cpp/build && build_mex() を実行してください\n');
else
    for i = 1:length(mex_files)
        fprintf('  %s: %,d bytes\n', mex_files(i).name, mex_files(i).bytes);
        % MD5 is helpful but not critical
    end
end
```

**実施** (オプション):
```bash
# 既存の diagnose_environment.m を確認し、必要に応じて追加
# git add kalman/diagnose_environment.m
# git commit -m "refactor: add binary metadata check to diagnose_environment"
```

### 4. 複数PC での動作確認

**目的**: このPC ではすでに確認済みなので、Other PC で実施

**Other PC での実施手順**:
```matlab
% 1. クローン＆セットアップ
git clone <repo>
cd KalmanFilter/kalman

% 2. 環境診断
diagnose_environment();
% 期待: TEST 1-5 すべて PASS（TEST 2 は バイナリ再ビルド後）

% 3. MEX ビルド
cd cpp/build
build_mex_verbose();
% 出力: build_env_report_YYYYMMDD_HHMMSS.txt が生成される

% 4. シミュレーション実行
cd ../..
run_simulation(42, true);
% 期待: Results/estimation_01.csv が生成される

% 5. 結果比較（このPC のデータがある場合）
% run_batch_10sets(); で10回実行して統計確認
```

**データ収集**:
```
Other PC で以下を記録:
- diagnose_environment() の出力
- build_env_report_*.txt の内容
- run_simulation(42, true) の実行時間
- Results/estimation_01.csv の Position RMSE
```

### ✅ **中期チェックリスト 完了後の状態**

```
repo/
├─ .gitignore ✓ (*.mexw64 含有確認)
├─ README.md ✓ (セットアップ手順追加)
├─ docs/
│  ├─ BINARY_MANAGEMENT.md ✓ (新規作成)
│  └─ ...
├─ kalman/
│  ├─ cpp/
│  │  ├─ build/
│  │  │  ├─ build_mex.m
│  │  │  ├─ build_mex_verbose.m ✓ (新規作成)
│  │  │  └─ build_env_report_*.txt (実行時生成)
│  │  ├─ bin/
│  │  │  ├─ mex_run_eskf.mexw64 ✓ (git管理外、各環境で生成)
│  │  │  └─ ...
│  │  └─ ...
│  ├─ diagnose_environment.m ✓ (拡張オプション)
│  └─ ...
```

---

## 📅 **長期チェックリスト（1ヶ月以内）** — CI/CD 統合

### 1. GitHub Actions ワークフロー（オプション — 高度な設定）

**目的**: 複数環境・MATLAB version で自動ビルド＆テスト

**ファイル**: `.github/workflows/mex_build_windows.yml`

**内容テンプレート**:
```yaml
name: Build MEX Binaries

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  build:
    runs-on: windows-latest
    
    strategy:
      matrix:
        matlab-version: ['R2024b', 'R2025a', 'R2025b']
    
    steps:
      - uses: actions/checkout@v3
      
      - name: Setup MATLAB
        uses: matlab-actions/setup-matlab@v1
        with:
          release: ${{ matrix.matlab-version }}
      
      - name: Build MEX
        run: |
          cd kalman\cpp\build
          matlab -batch "build_mex(); exit"
      
      - name: Run Tests
        run: |
          cd kalman
          matlab -batch "run_batch_10sets(); exit"
      
      - name: Upload Artifacts
        uses: actions/upload-artifact@v3
        with:
          name: mex-${{ matrix.matlab-version }}
          path: kalman/cpp/bin/*.mexw64
      
      - name: Create Release (on tag)
        if: startsWith(github.ref, 'refs/tags/')
        uses: softprops/action-gh-release@v1
        with:
          files: kalman/cpp/bin/*.mexw64
```

**実施** (オプション):
```bash
# .github/workflows/ ディレクトリを作成
mkdir -p .github/workflows

# mex_build_windows.yml を作成・配置
# git add .github/workflows/mex_build_windows.yml
# git commit -m "ci: add GitHub Actions workflow for MEX building"
```

### 2. Release Strategy（オプション）

**目的**: GitHub Release に プリコンパイル済みバイナリを配置

**タグ付けと Release の作成**:
```bash
git tag -a v1.0.0 -m "First release with precompiled binaries"
git push origin v1.0.0

# GitHub web UI で Release ページを開き、
# kalman/cpp/bin/*.mexw64 をアップロード
```

**ユーザーへの案内**:
```markdown
## ダウンロード

3つの選択肢があります:

1. **ローカルビルド（推奨）**
   ```matlab
   cd kalman/cpp/build
   build_mex();
   ```

2. **Release からダウンロード**
   https://github.com/your-org/KalmanFilter/releases
   → .mexw64 ファイルをダウンロードして kalman/cpp/bin/ に配置

3. **最新開発版（CI/CD Artifacts）**
   各 commit の CI run から成果物を取得
```

### 3. Docker イメージ（オプション — エンタープライズ向け）

**ファイル**: `Dockerfile`

**内容テンプレート**:
```dockerfile
FROM mcr.microsoft.com/windows/servercore:ltsc2022

# MATLAB + Visual Studio インストール
RUN powershell -NoProfile -ExecutionPolicy Bypass -Command \
    choco install -y matlab visualstudio2022buildtools

# リポジトリクローン
WORKDIR /src
RUN git clone https://github.com/your-org/KalmanFilter.git

# MEX ビルド
WORKDIR /src/KalmanFilter/kalman/cpp/build
RUN matlab -batch "build_mex(); exit"

# 成果物をエクスポート
RUN copy "C:\src\KalmanFilter\kalman\cpp\bin\*.mexw64" \
         "C:\output\"
```

**実施**:
```bash
# Docker イメージをビルド
docker build -t kalmanfilter:latest .

# バイナリを抽出
docker run --rm -v "C:\output:/output" kalmanfilter:latest
# → C:\output に .mexw64 が生成される
```

### ✅ **長期チェックリスト 目標状態**

```
GitHub Actions による自動ビルド:
  ✓ 複数 MATLAB version でテスト
  ✓ Windows + Linux をサポート（将来）
  ✓ PR マージ時に自動ビルド＆テスト
  ✓ Release に自動アップロード

ユーザーの選択肢:
  A. ローカルビルド（推奨、高速）
  B. Release download（依存性最小）
  C. Docker（完全な環境制御）
```

---

## 📋 **チェックリスト確認用テンプレート**

### 自分のPC で実施した確認

```
【短期】
☐ .gitignore に *.mexw64 が含まれている (git check-ignore kalman/cpp/bin/*.mexw64)
☐ README.md に セットアップ手順を追加
☐ git status が clean である (git status --porcelain)

【中期】
☐ build_mex_verbose.m が kalman/cpp/build/ に存在する
☐ BINARY_MANAGEMENT.md が docs/ に存在する
☐ diagnose_environment() を実行して 5/5 テスト PASS
☐ run_simulation(1, true) を実行して Results/estimation_01.csv 生成 PASS

【長期】(オプション)
☐ .github/workflows/mex_build_windows.yml を作成
☐ GitHub Release に .mexw64 をアップロード
☐ Docker イメージをテスト
```

### Other PC で実施した確認

```
【初期セットアップ】
☐ Git clone ＆ cd KalmanFilter/kalman
☐ diagnose_environment() で TEST 1-5 が PASS (TEST 2は未初期化で失敗OK)
☐ cd cpp/build && build_mex() を実行
☐ build_mex_verbose() で環境情報ログを確認
☐ run_simulation(42, true) を実行して Results/estimation_01.csv 生成 PASS

【バイナリ確認】
☐ ls -lh cpp/bin/*.mexw64 で サイズを確認
☐ md5sum cpp/bin/*.mexw64 で ハッシュを記録
☐ build_env_report_*.txt が cpp/build/ に生成された

【結果比較】
☐ compare_environments.m で This PC との差を評価
  → RMSE < 1e-3 であれば正常
  → RMSE >= 0.01 であれば、バイナリまたはデータに問題
```

---

## 🔗 **関連ドキュメント**

- [BINARY_MANAGEMENT.md](./BINARY_MANAGEMENT.md) — 詳細な技術解説
- [diagnose_environment.m](../kalman/diagnose_environment.m) — 環境診断スクリプト
- [build_mex_verbose.m](../kalman/cpp/build/build_mex_verbose.m) — 詳細ビルドログ出力
- [analyze_build_differences.m](../kalman/analyze_build_differences.m) — バイナリ差異分析

---

## ✨ **まとめ**

| 段階 | 実施内容 | 難易度 | 時間 |
|------|---------|--------|------|
| **短期** | .gitignore + README 更新 | ⭐ | 15分 |
| **中期** | build_mex_verbose + 複数PC確認 | ⭐⭐ | 1-2時間 |
| **長期** | GitHub Actions + Release | ⭐⭐⭐ | 4-8時間 |

**今日のミッション**: ✅ **短期を完了する**  
→ これで、複数PC での開発準備が整います

