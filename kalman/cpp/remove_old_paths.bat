@echo off
REM 旧パスディレクトリの削除スクリプト
REM Inc/, Src/, Lib/以外の古いディレクトリを削除

cd /d "%~dp0"

echo Removing old path directories...
echo.
echo WARNING: This will delete the following directories:
echo   - Common/
echo   - EKF/
echo   - ESKF/
echo   - KF/
echo   - MEUKF/
echo   - include/ (lowercase)
echo   - src/ (lowercase)
echo.
pause

if exist Common (
    echo Removing Common/...
    rmdir /s /q Common
)
if exist EKF (
    echo Removing EKF/...
    rmdir /s /q EKF
)
if exist ESKF (
    echo Removing ESKF/...
    rmdir /s /q ESKF
)
if exist KF (
    echo Removing KF/...
    rmdir /s /q KF
)
if exist MEUKF (
    echo Removing MEUKF/...
    rmdir /s /q MEUKF
)
if exist include (
    echo Removing include/ (lowercase)...
    rmdir /s /q include
)
if exist src (
    echo Removing src/ (lowercase)...
    rmdir /s /q src
)

echo.
echo Done.
echo.
echo Remaining directories:
echo   - Inc/ (new include directory)
echo   - Src/ (new source directory)
echo   - Lib/ (independent libraries)
echo   - MEX/ (MEX wrappers)
echo   - build/ (build scripts)
echo   - bin/ (build output)
pause

