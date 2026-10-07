@echo off
setlocal DisableDelayedExpansion
rem The shortcut supplies absolute runtime paths; CLI callers may set them too.
if not defined MMHAND_PYTHON if exist "%~dp0.venv\Scripts\python.exe" set "MMHAND_PYTHON=%~dp0.venv\Scripts\python.exe"
if not defined MMHAND_PYTHON if defined VIRTUAL_ENV if exist "%VIRTUAL_ENV%\Scripts\python.exe" set "MMHAND_PYTHON=%VIRTUAL_ENV%\Scripts\python.exe"
if not defined MMHAND_PYTHON for /f "delims=" %%P in ('py -3 -c "import sys; print(sys.executable)" 2^>nul') do set "MMHAND_PYTHON=%%P"
if not defined MMHAND_PYTHON for /f "delims=" %%P in ('python -c "import sys; print(sys.executable)" 2^>nul') do set "MMHAND_PYTHON=%%P"
if not exist "%MMHAND_PYTHON%" (
    echo Python not found: "%MMHAND_PYTHON%"
    echo Recreate the shortcut with -PythonPath or set MMHAND_PYTHON.
    goto :failed
)
if defined MMHAND_ROS_SETUP (
    if not exist "%MMHAND_ROS_SETUP%" (
        echo ROS2 setup not found: "%MMHAND_ROS_SETUP%"
        goto :failed
    )
    call "%MMHAND_ROS_SETUP%"
    if errorlevel 1 goto :failed
)
pushd "%~dp0" || goto :failed
"%MMHAND_PYTHON%" -m desktop %*
set "result=%errorlevel%"
popd
if "%result%"=="0" exit /b 0
:failed
echo MMHand startup or execution failed. See the error above.
rem Set only by automated checks to avoid waiting for keyboard input.
if not defined MMHAND_NO_PAUSE pause
exit /b 1
