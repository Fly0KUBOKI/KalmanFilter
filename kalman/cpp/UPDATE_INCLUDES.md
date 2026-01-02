# Includeパス一括更新スクリプト

以下のPowerShellスクリプトを実行すると、すべてのファイルのincludeパスを一括更新できます。

## 実行方法

```powershell
cd kalman/cpp

# Incフォルダ内のファイルを更新
Get-ChildItem -Path Inc -Recurse -Filter *.hpp | ForEach-Object {
    $content = Get-Content $_.FullName -Raw
    $content = $content -replace '#include\s+"Common/Math/fixed_matrix\.hpp"', '#include "Lib/Matrix/fixed_matrix.hpp"'
    $content = $content -replace '#include\s+"Common/Math/quaternion\.hpp"', '#include "Lib/Quaternion/quaternion_functions.hpp"'
    $content = $content -replace '#include\s+"Common/Math/quaternion_lib\.hpp"', '#include "Lib/Quaternion/quaternion_lib.hpp"'
    Set-Content -Path $_.FullName -Value $content -NoNewline
}

# Srcフォルダ内のファイルを更新
Get-ChildItem -Path src -Recurse -Filter *.cpp | ForEach-Object {
    $content = Get-Content $_.FullName -Raw
    $content = $content -replace '#include\s+"\.\.\/\.\.\/Inc\/Common\/Math\/fixed_matrix\.hpp"', '#include "Lib/Matrix/fixed_matrix.hpp"'
    $content = $content -replace '#include\s+"\.\.\/\.\.\/Inc\/Common\/Math\/quaternion\.hpp"', '#include "Lib/Quaternion/quaternion_functions.hpp"'
    $content = $content -replace '#include\s+"\.\.\/\.\.\/Inc\/Common\/Math\/quaternion_lib\.hpp"', '#include "Lib/Quaternion/quaternion_lib.hpp"'
    $content = $content -replace '#include\s+"Common\/Math\/quaternion_lib\.hpp"', '#include "Lib/Quaternion/quaternion_lib.hpp"'
    Set-Content -Path $_.FullName -Value $content -NoNewline
}

# MEX/Incフォルダ内のファイルを更新
Get-ChildItem -Path MEX/Inc -Filter *.hpp | ForEach-Object {
    $content = Get-Content $_.FullName -Raw
    $content = $content -replace '#include\s+"\.\.\/\.\.\/Inc\/Common\/Math\/fixed_matrix\.hpp"', '#include "Lib/Matrix/fixed_matrix.hpp"'
    $content = $content -replace '#include\s+"\.\.\/\.\.\/Inc\/Common\/Math\/quaternion\.hpp"', '#include "Lib/Quaternion/quaternion_functions.hpp"'
    $content = $content -replace '#include\s+"\.\.\/\.\.\/Inc\/Common\/Math\/quaternion_lib\.hpp"', '#include "Lib/Quaternion/quaternion_lib.hpp"'
    Set-Content -Path $_.FullName -Value $content -NoNewline
}

# UKFフォルダ内のファイルを更新
$ukfFile = "src/UKF/ukf_sigma_points.cpp"
if (Test-Path $ukfFile) {
    $content = Get-Content $ukfFile -Raw
    $content = $content -replace '#include\s+"\.\.\/\.\.\/Inc\/Common\/Math\/fixed_matrix\.hpp"', '#include "Lib/Matrix/fixed_matrix.hpp"'
    Set-Content -Path $ukfFile -Value $content -NoNewline
}
```

実行後、以下のコマンドで更新状況を確認できます：

```powershell
Get-ChildItem -Recurse -Include *.hpp,*.cpp | Select-String -Pattern 'Common/Math/(fixed_matrix|quaternion)' | Select-Object Path, LineNumber, Line
```

