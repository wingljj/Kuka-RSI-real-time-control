@echo off
chcp 65001 >nul
title RSI 帧捕获
rem -u 关闭 Python 的 stdout 缓冲，保证控制台逐帧实时打印
python -u "%~dp0udp_capture.py"
pause
