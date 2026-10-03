@echo off
rem SpaceCalibrator518 build: configure with the VS 2022 generator, build RelWithDebInfo, assemble dist\.
rem Requires Visual Studio 2022 Build Tools (C++ workload), CMake 3.24+ and the submodules checked out.
setlocal
cd /d "%~dp0"
if not exist logs mkdir logs
echo [1/3] configure
cmake -G "Visual Studio 17 2022" -A x64 -B bin -S . > logs\cmake-configure.log 2>&1 || (type logs\cmake-configure.log & exit /b 1)
echo [2/3] build (RelWithDebInfo)
cmake --build bin --config RelWithDebInfo --parallel > logs\cmake-build.log 2>&1 || (findstr /i /c:"error" logs\cmake-build.log & exit /b 1)
echo [3/3] install (live overlay swap while SteamVR runs, see tools\install.ps1)
powershell -NoProfile -ExecutionPolicy Bypass -File tools\install.ps1 %* || exit /b 1
echo done: see logs\install.log
endlocal
