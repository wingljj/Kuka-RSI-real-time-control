@echo off
chcp 65001 >nul
title RSI 链路诊断
rem 不用 -File + -ExecutionPolicy Bypass：那会覆盖本机的脚本执行策略。
rem 执行策略只约束脚本文件的直接调用，读进来再执行不受其约束，
rem 顺带能显式按 UTF-8 解码，避免中文在管道里被代码页切碎。
powershell -NoProfile -Command "Invoke-Expression ([IO.File]::ReadAllText('%~dp0网络诊断.ps1', [Text.Encoding]::UTF8))"
