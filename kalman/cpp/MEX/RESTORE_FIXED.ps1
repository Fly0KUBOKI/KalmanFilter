# RESTORE_FIXED.ps1
# Gitから紛失したソースファイルを復元するスクリプト（修正版）

# 各ファイルについて、実際に存在するコミットとパスを確認してから復元

Write-Host "=== Restoring missing source files ===" -ForegroundColor Cyan
Write-Host ""

# mex_eskf_predict_postprocess.cpp
Write-Host "1. Checking mex_eskf_predict_postprocess.cpp..." -ForegroundColor Yellow
$log1 = git log --all --full-history --oneline -- "kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp"
if ($log1) {
    $commits1 = $log1 | ForEach-Object { $_.Split(' ')[0] }
    foreach ($commit in $commits1) {
        Write-Host "  Trying commit: $commit"
        # コミット内のファイル一覧を確認
        $files = git ls-tree -r $commit --name-only | Select-String -Pattern "predict_postprocess"
        if ($files) {
            Write-Host "  Found in commit $commit at path: $files" -ForegroundColor Green
            $path = $files[0]
            git show "${commit}:${path}" | Out-File -Encoding utf8 "kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp"
            Write-Host "  Restored: mex_eskf_predict_postprocess.cpp" -ForegroundColor Green
            break
        }
    }
} else {
    Write-Host "  Not found in git history" -ForegroundColor Red
}
Write-Host ""

# mex_eskf_update_postprocess.cpp
Write-Host "2. Checking mex_eskf_update_postprocess.cpp..." -ForegroundColor Yellow
$log2 = git log --all --full-history --oneline -- "kalman/cpp/MEX/mex_eskf_update_postprocess.cpp"
if ($log2) {
    $commits2 = $log2 | ForEach-Object { $_.Split(' ')[0] }
    foreach ($commit in $commits2) {
        Write-Host "  Trying commit: $commit"
        $files = git ls-tree -r $commit --name-only | Select-String -Pattern "update_postprocess"
        if ($files) {
            Write-Host "  Found in commit $commit at path: $files" -ForegroundColor Green
            $path = $files[0]
            git show "${commit}:${path}" | Out-File -Encoding utf8 "kalman/cpp/MEX/mex_eskf_update_postprocess.cpp"
            Write-Host "  Restored: mex_eskf_update_postprocess.cpp" -ForegroundColor Green
            break
        }
    }
} else {
    Write-Host "  Not found in git history" -ForegroundColor Red
}
Write-Host ""

# mex_meukf_step.cpp
Write-Host "3. Checking mex_meukf_step.cpp..." -ForegroundColor Yellow
$log3 = git log --all --full-history --oneline -- "kalman/cpp/MEX/mex_meukf_step.cpp"
if ($log3) {
    $commits3 = $log3 | ForEach-Object { $_.Split(' ')[0] }
    foreach ($commit in $commits3) {
        Write-Host "  Trying commit: $commit"
        $files = git ls-tree -r $commit --name-only | Select-String -Pattern "meukf_step"
        if ($files) {
            Write-Host "  Found in commit $commit at path: $files" -ForegroundColor Green
            $path = $files[0]
            git show "${commit}:${path}" | Out-File -Encoding utf8 "kalman/cpp/MEX/mex_meukf_step.cpp"
            Write-Host "  Restored: mex_meukf_step.cpp" -ForegroundColor Green
            break
        }
    }
} else {
    Write-Host "  Not found in git history" -ForegroundColor Red
}
Write-Host ""

Write-Host "=== Done ===" -ForegroundColor Cyan

