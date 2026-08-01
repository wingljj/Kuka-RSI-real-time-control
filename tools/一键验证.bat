@echo off
chcp 65001 >nul
title KUKA RSI 一键验证
set "PATH=D:\Software\QT\content\6.5.3\mingw_64\bin;%PATH%"
cd /d "%~dp0.."

rem 定位 Git Bash（验证脚本是 bash 写的）
set "BASH=C:\Program Files\Git\bin\bash.exe"
if not exist "%BASH%" set "BASH=C:\Program Files\Git\usr\bin\bash.exe"
if not exist "%BASH%" (
    echo 未找到 Git Bash，请安装 Git for Windows 后重试。
    pause
    exit /b 1
)

echo ============================================================
echo  KR210 真实约束端到端验证（自检/闭环/限位/速度/会话重启）
echo ============================================================
"%BASH%" tools/verify_kinematics.sh
echo.

echo ============================================================
echo  通信健壮性回归（丢包/重复/跳号/乱序/SENTYPE 错配/延迟保护）
echo ============================================================
"%BASH%" tools/verify_robustness.sh
echo.

echo 验证完成。两项都应显示 PASS=x FAIL=0。
pause
