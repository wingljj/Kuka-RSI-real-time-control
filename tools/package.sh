#!/usr/bin/env bash
# 打包成自包含的 dist/ 目录：Qt 运行库都在里面，目标机不需要装 Qt。
# RL 的运行时依赖（libxml2 及其传递依赖）与模型（robot.rlmdl.xml）一并打进。
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
MINGW_WIN="D:/Software/QT/content/Tools/mingw1120_64/bin"
CMAKE="D:/Software/QT/content/Tools/CMake_64/bin/cmake.exe"
# PATH 里的各项必须是 POSIX 形式：MSYS 按 ':' 切分 PATH，
# Windows 形式的 D:/... 会被撕成两个无效项并静默选中别的编译器。
# ucrt64/bin 里的 libxml2 链是 RL 的运行时依赖；configure 阶段
# FindIconv 的 check_c_source_runs 要真编译运行，也必须它们在 PATH。
MINGW="/d/Software/QT/content/Tools/mingw1120_64/bin"
NINJA="/d/Software/QT/content/Tools/Ninja"
QTBIN="/d/Software/QT/content/6.5.3/mingw_64/bin"
UCRT="/c/msys64/ucrt64/bin"
BUILD_PATH="$MINGW:$NINJA:$QTBIN:$UCRT:/c/msys64/usr/bin:$PATH"

echo "==> Release 构建"
# RL 配置钉（docs/rl-build-notes.md）：rl_DIR 指向 MinGW 静态构建树
# （小写！），屏蔽注册表里的 MSVC 版 RL；Eigen/Boost 显式指到本机路径
# （Boost_NO_SYSTEM_PATHS 屏蔽官方 RL 包自带的 boost）；CMAKE_PREFIX_PATH
# 加 ucrt64 让 libxml2/iconv/lzma/zlib 可见；编译器显式指回 Qt 的 mingw
# 工具链，避免 CMAKE_PREFIX_PATH 里的 ucrt64 把 windres 换成 UCRT 版。
PATH="$BUILD_PATH" "$CMAKE" -S . -B build-release -G Ninja \
  -DCMAKE_PREFIX_PATH="$QTDIR_WIN;C:/msys64/ucrt64" \
  -DCMAKE_BUILD_TYPE=Release \
  -Drl_DIR=D:/Software/rl-build/lib/cmake/rl \
  -DCMAKE_FIND_USE_REGISTRY=OFF \
  -DEIGEN3_INCLUDE_DIR=E:/download/eigen-3.3.8/eigen-3.3.8 \
  -DBOOST_ROOT=D:/Software/boost_1_85_0/boost_1_85_0 \
  -DBoost_INCLUDE_DIR=D:/Software/boost_1_85_0/boost_1_85_0 \
  -DBoost_NO_SYSTEM_PATHS=ON \
  -DCMAKE_C_COMPILER="$MINGW_WIN/gcc.exe" \
  -DCMAKE_CXX_COMPILER="$MINGW_WIN/g++.exe" \
  -DCMAKE_RC_COMPILER="$MINGW_WIN/windres.exe"
PATH="$BUILD_PATH" "$CMAKE" --build build-release

echo "==> 组装 dist/"
rm -rf dist
mkdir -p dist/config dist/krc dist/platforms dist/styles dist/imageformats
cp build-release/rsi_host.exe                        dist/
cp build-release/tools/krc_simulator/krc_simulator.exe dist/
cp config/rsi_config.json                            dist/config/
cp -r krc/.                                          dist/krc/   # 含 legacy-krc4/
# 链路诊断（判断 KRC 有没有把帧发到宿主）；.bat 只是 .ps1 的启动壳，两个都要
cp tools/网络诊断.ps1 tools/网络诊断.bat  dist/
cp tools/udp_capture.py tools/网络抓包.bat      dist/
# 免安装启动器：cd 到 dist 自身目录后以相对路径 --model robot.rlmdl.xml 运行。
# main.cpp 的默认 --model 是开发机绝对路径，分发包里必须显式传相对路径。
cp tools/启动模拟器-dist.bat                    dist/启动模拟器.bat

echo "==> RL 模型（Comau Racer 7-1.4，--model 指定路径）"
cp "D:/QTproj/rl/rl-master/3dmodel/robot.rlmdl.xml" dist/robot.rlmdl.xml

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

echo "==> RL 运行库（libxml2 及传递依赖 libiconv/liblzma/zlib）"
# RL 的 math/mdl/kin/xml 静态链接进 exe，但 xml 模块仍动态依赖 libxml2；
# libxml2 又依赖 iconv/lzma/zlib（ldd 实测，三者只依赖系统 DLL）。
# 缺 libxml2-2.dll 时 exe 起不来，缺其余三个时运行到模型加载处崩。
for d in libxml2-2.dll libiconv-2.dll liblzma-5.dll zlib1.dll; do
  cp "$UCRT/$d" dist/
done

echo "==> 完成：$(du -sh dist | cut -f1)"
echo "    验证方式：在不设任何 Qt/RL 路径的 shell 里运行 dist/rsi_host.exe，"
echo "    界面能起来才算数——带着 Qt 的 PATH 测等于没测。"
echo "    模拟器：dist/启动模拟器.bat（带 --model robot.rlmdl.xml），"
echo "    或 dist/krc_simulator.exe --model robot.rlmdl.xml --self-test。"
