# Gitコマンド集

## 現在の変更を確認
```powershell
cd kalman\cpp
git status
```

## 変更を一時的に保存（stash）
```powershell
cd kalman\cpp
git stash
```

## 特定のコミットまで戻る（指定のコミットハッシュが存在する場合）
```powershell
cd kalman\cpp
git log --oneline --all
# コミットハッシュを確認後、以下を実行
git checkout <コミットハッシュ>
```

## 特定のコミットまで戻る（新しいブランチを作成）
```powershell
cd kalman\cpp
git checkout -b restore_commit <コミットハッシュ>
```

## 現在の変更を破棄して元に戻す
```powershell
cd kalman\cpp
git restore .
# または
git checkout .
```

## 特定のファイルを元に戻す
```powershell
cd kalman\cpp
git restore <ファイル名>
# 例: git restore MEX/mex_run_eskf.cpp
```

## コミット履歴を確認
```powershell
cd kalman\cpp
git log --oneline -20
```

## 特定のコミットの内容を確認
```powershell
cd kalman\cpp
git show <コミットハッシュ>
```

## コミットハッシュで検索（UUID形式の場合）
```powershell
cd kalman\cpp
git log --all --oneline | Select-String "a0c474c3"
```

## 現在の変更を確認（差分表示）
```powershell
cd kalman\cpp
git diff
```

## 特定のファイルの変更を確認
```powershell
cd kalman\cpp
git diff <ファイル名>
```


