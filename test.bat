@echo off
rem Builds and runs the fork's C++ tests, then the Python tests in the venv.
setlocal
cd /d "%~dp0"
if not exist logs mkdir logs
cmake -G "Visual Studio 17 2022" -A x64 -B bin-tests -S . -DSPACECAL_BUILD_TESTS=ON > logs\cmake-tests-configure.log 2>&1 || (type logs\cmake-tests-configure.log & exit /b 1)
cmake --build bin-tests --config Debug --target spacecal_tests --parallel > logs\cmake-tests-build.log 2>&1 || (findstr /i /c:"error" logs\cmake-tests-build.log & exit /b 1)
rem the C++ test records a fresh event fixture; the Python tests read that one (the committed
rem fixture under tools\tests\fixtures stays untouched so the tree stays clean)
set SPACECAL_TEST_FIXTURE_OUT=%~dp0bin-tests\fixture\event_sample
set SPACECAL_TEST_FIXTURE=%~dp0bin-tests\fixture\event_sample
bin-tests\artifacts\Debug\spacecal_blackbox_test.exe || exit /b 1
bin-tests\artifacts\Debug\spacecal_trust_test.exe || exit /b 1
bin-tests\artifacts\Debug\spacecal_replay_test.exe || exit /b 1
bin-tests\artifacts\Debug\spacecal_pose_ring_test.exe || exit /b 1
bin-tests\artifacts\Debug\spacecal_live_test.exe || exit /b 1
bin-tests\artifacts\Debug\spacecal_universe_test.exe || exit /b 1
bin-tests\artifacts\Debug\spacecal_frames_test.exe || exit /b 1
echo replay tool: bin-tests\artifacts\Debug\spacecal-replay.exe
if exist .venv\Scripts\python.exe (
    .venv\Scripts\python.exe -m pytest tools\tests -q || exit /b 1
) else (
    echo .venv missing: uv venv .venv --python 3.12 ^&^& .venv\Scripts\python -m pip install -r tools\requirements.txt
)
endlocal
