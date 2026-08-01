@echo off
chcp 65001 >nul
title KUKA RSI 模拟器（RL 运动学 · 免安装版）
rem 免安装版启动器：dist/ 自带 Qt / MinGW / libxml2 运行库与 RL 模型
rem robot.rlmdl.xml，目标机不需要安装 Qt、不需要设置任何环境变量。
rem 模型通过 --model 指定（相对本目录的相对路径），可换成任意 rlmdl 文件。
rem 先 cd 到自身目录再运行：--model robot.rlmdl.xml 按相对路径解析。
cd /d "%~dp0"

echo ============================================================
echo  KUKA RSI 模拟器（RL 运动学 · 免安装版）
echo  模型：robot.rlmdl.xml（--model 可覆盖）
echo  向 127.0.0.1:59152 发帧 —— 请先启动 rsi_host.exe，
echo  并在界面上把监听地址改成 127.0.0.1、点「开始监听」。
echo.
echo  可在命令末尾追加参数，常用：
echo    --max-vel-pos 20 --max-accel-pos 2000   速度/加速度限制
echo    --joint-limits "0 0 -60 -60 30 30 0 0 90 90 0 0"   关节限位钉死
echo    --cycles N             发帧数（0 = 一直运行，默认；Ctrl+C 停止）
echo ============================================================
echo.

"%~dp0krc_simulator.exe" --model robot.rlmdl.xml --host 127.0.0.1 --port 59152 --cycles 0 %*

echo.
echo 运行结束（若想跑不同参数，改上面的命令或双击后追加）。
pause
