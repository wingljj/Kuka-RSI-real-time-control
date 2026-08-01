@echo off
chcp 65001 >nul
title KUKA KR210 RSI 模拟器
rem 设置 Qt 运行库路径（模拟器依赖 Qt6Core / Qt6Network）
set "PATH=D:\Software\QT\content\6.5.3\mingw_64\bin;%PATH%"
cd /d "%~dp0.."

echo ============================================================
echo  KUKA KR210 RSI 模拟器
echo  向 127.0.0.1:59152 发帧 —— 请先启动 rsi_host.exe，
echo  并在界面上把监听地址改成 127.0.0.1、点「开始监听」。
echo.
echo  可在命令末尾追加参数，常用：
echo    --max-vel-pos 20 --max-accel-pos 2000   速度/加速度限制
echo    --restart-at-ms 3000 --restart-gap-ms 2200   会话重启
echo    --joint-limits "0 0 -60 -60 30 30 0 0 90 90 0 0"   关节限位钉死
echo    --ipoc-wrap-at 1012    IPOC 回绕
echo    --cycles N             发帧数（0 = 一直运行，默认；Ctrl+C 停止）
echo ============================================================
echo.

"%~dp0..\build\tools\krc_simulator\krc_simulator.exe" --host 127.0.0.1 --port 59152 --cycles 0 %*

echo.
echo 运行结束（若想跑不同参数，改上面的命令或双击后追加）。
pause
