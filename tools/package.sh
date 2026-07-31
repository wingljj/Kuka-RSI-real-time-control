#!/usr/bin/env bash
# 打包成自包含的 dist/ 目录：Qt 运行库都在里面，目标机不需要装 Qt。
#
# 用法：bash tools/package.sh
#
# 注意 windeployqt 在本机不可用：该 Qt 安装的 plugins/platforms/ 目录里
# 混进了一个 rlPlanDemo.exe，windeployqt 扫描时会报 "Unable to find the
# platform plugin" 并跳过全部 GUI 依赖，只复制 Core/Network，得到一个
# 启动就崩的包。所以这里手工复制，清单是 windeployqt 自己报出来的
# Direct dependencies 加上平台插件。
set -e
cd "$(dirname "$0")/.."

QTDIR_WIN="D:/Software/QT/content/6.5.3/mingw_64"
QTDIR="/d/Software/QT/content/6.5.3/mingw_64"
CMAKE="D:/Software/QT/content/Tools/CMake_64/bin/cmake.exe"
# PATH 里的这两项必须是 POSIX 形式：MSYS 按 ':' 切分 PATH，
# Windows 形式的 D:/... 会被撕成两个无效项并静默选中别的编译器。
MINGW="/d/Software/QT/content/Tools/mingw1120_64/bin"
NINJA="/d/Software/QT/content/Tools/Ninja"

echo "==> Release 构建"
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" -S . -B build-release -G Ninja \
  -DCMAKE_PREFIX_PATH="$QTDIR_WIN" -DCMAKE_BUILD_TYPE=Release
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build-release

echo "==> 组装 dist/"
rm -rf dist
mkdir -p dist/config dist/krc dist/platforms dist/styles dist/imageformats
cp build-release/rsi_host.exe                        dist/
cp build-release/tools/krc_simulator/krc_simulator.exe dist/
cp config/rsi_config.json                            dist/config/
cp krc/*                                             dist/krc/
# 链路诊断（判断 KRC 有没有把帧发到宿主）；.bat 只是 .ps1 的启动壳，两个都要
cp tools/网络诊断.ps1 tools/网络诊断.bat  dist/

echo "==> Qt 运行库"
for d in Qt6Core Qt6Gui Qt6Widgets Qt6Network Qt6Charts Qt6OpenGL Qt6OpenGLWidgets; do
  cp "$QTDIR/bin/$d.dll" dist/
done
# 没有 qwindows.dll 的话 GUI 程序根本起不来
cp "$QTDIR/plugins/platforms/qwindows.dll"        dist/platforms/
cp "$QTDIR/plugins/styles/qwindowsvistastyle.dll" dist/styles/       2>/dev/null || true
cp "$QTDIR/plugins/imageformats/qico.dll"         dist/imageformats/ 2>/dev/null || true

echo "==> MinGW 运行库"
for d in libgcc_s_seh-1.dll libstdc++-6.dll libwinpthread-1.dll; do
  cp "$MINGW/$d" dist/
done

echo "==> 完成：$(du -sh dist | cut -f1)"
echo "    验证方式：在不设任何 Qt 路径的 shell 里运行 dist/rsi_host.exe，"
echo "    界面能起来才算数——带着 Qt 的 PATH 测等于没测。"
