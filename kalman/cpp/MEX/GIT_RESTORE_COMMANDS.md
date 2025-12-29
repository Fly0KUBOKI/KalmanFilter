# Gitでソースコードを復元するコマンド

## 紛失しているソースファイル

以下のファイルはバイナリが存在するが、ソースコードが紛失しています：

1. `mex_eskf_predict_postprocess.cpp`
2. `mex_eskf_update_postprocess.cpp`
3. `mex_eskf_zupt.cpp`
4. `mex_meukf_step.cpp`

## Gitでファイルを探すコマンド

### 1. ファイルの履歴を確認する

```bash
# ファイルが存在していた最後のコミットを探す
git log --all --full-history --oneline -- "kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp"
git log --all --full-history --oneline -- "kalman/cpp/MEX/mex_eskf_update_postprocess.cpp"
git log --all --full-history --oneline -- "kalman/cpp/MEX/mex_eskf_zupt.cpp"
git log --all --full-history --oneline -- "kalman/cpp/MEX/mex_meukf_step.cpp"
```

### 2. ファイルが存在していた最後のコミットを特定する

```bash
# より詳細な情報を取得
git log --all --full-history --diff-filter=D --summary -- "kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp"
git log --all --full-history --diff-filter=D --summary -- "kalman/cpp/MEX/mex_eskf_update_postprocess.cpp"
git log --all --full-history --diff-filter=D --summary -- "kalman/cpp/MEX/mex_eskf_zupt.cpp"
git log --all --full-history --diff-filter=D --summary -- "kalman/cpp/MEX/mex_meukf_step.cpp"
```

### 3. ファイルを復元する

#### 方法1: 最後に存在していたコミットから復元

```bash
# 最後に存在していたコミットハッシュを取得（上記のコマンドで確認）
# 例: コミットハッシュが abc123 の場合

# ファイルを確認
git show abc123:kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp

# ファイルを復元
git show abc123:kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp > kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp
git show abc123:kalman/cpp/MEX/mex_eskf_update_postprocess.cpp > kalman/cpp/MEX/mex_eskf_update_postprocess.cpp
git show abc123:kalman/cpp/MEX/mex_eskf_zupt.cpp > kalman/cpp/MEX/mex_eskf_zupt.cpp
git show abc123:kalman/cpp/MEX/mex_meukf_step.cpp > kalman/cpp/MEX/mex_meukf_step.cpp
```

#### 方法2: 削除されたコミットの直前のコミットから復元

```bash
# ファイルが削除されたコミットを特定
git log --all --full-history --diff-filter=D -- "kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp"

# 削除されたコミットの直前（親コミット）から復元
# 例: 削除されたコミットが def456 の場合、その親コミット abc123 から復元
git show def456^:kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp > kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp
```

#### 方法3: すべてのブランチで検索して復元

```bash
# すべてのブランチでファイルを検索
git log --all --full-history --source -- "kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp"

# 見つかったコミットから復元
git show <commit-hash>:kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp > kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp
```

### 4. 一括で復元するスクリプト

```bash
#!/bin/bash
# restore_missing_sources.sh

FILES=(
    "kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp"
    "kalman/cpp/MEX/mex_eskf_update_postprocess.cpp"
    "kalman/cpp/MEX/mex_eskf_zupt.cpp"
    "kalman/cpp/MEX/mex_meukf_step.cpp"
)

for file in "${FILES[@]}"; do
    echo "Searching for: $file"
    
    # 最後に存在していたコミットを探す
    COMMIT=$(git log --all --full-history --oneline -- "$file" | head -1 | awk '{print $1}')
    
    if [ -n "$COMMIT" ]; then
        echo "Found in commit: $COMMIT"
        echo "Restoring $file..."
        git show "$COMMIT:$file" > "$file"
        echo "Restored: $file"
    else
        echo "Not found in git history: $file"
    fi
    echo ""
done
```

## PowerShell版（Windows用）

```powershell
# ファイルの履歴を確認
git log --all --full-history --oneline -- "kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp"

# ファイルを復元
$commit = git log --all --full-history --oneline -- "kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp" | Select-Object -First 1 | ForEach-Object { $_.Split(' ')[0] }
git show "$commit:kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp" | Out-File -Encoding utf8 "kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp"
```

## 注意事項

1. **ファイルの存在確認**: 復元後、ファイルが正しく復元されたか確認してください
2. **エンコーディング**: WindowsではUTF-8エンコーディングで保存することを推奨
3. **コンパイル確認**: 復元後、`build_mex.m`でコンパイルできるか確認してください
4. **依存関係**: 復元したファイルが`mex_quaternion_lib`などの依存関係を正しく参照しているか確認してください

## 推奨手順

1. まず、各ファイルがgit履歴に存在するか確認
2. 存在する場合は、最後に存在していたコミットから復元
3. 復元後、コンパイルして動作確認
4. 問題がなければ、gitにコミット

