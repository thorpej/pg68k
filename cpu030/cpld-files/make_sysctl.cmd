@echo off

set VAR_FITTERDIR=C:\ATMEL_PLS_Tools\Prochip\pldfit
set VAR_NAME="sysctl"

set VAR_DEVICE=ATF1508AS
set VAR_PACKAGE=TQFP100
set VAR_SPEED=7
set VAR_FITTER=fit%VAR_DEVICE:~3,4%.exe

if not exist "%VAR_FITTERDIR%\atmel.std" (
	echo Unable to locate Atmel fitters
	exit /b 1
)

if exist "%VAR_NAME%.fit" del /f /q "%VAR_NAME%.fit">nul

echo %VAR_FITTERDIR%\%VAR_FITTER% ^
-i "%VAR_NAME%.edif" ^
-ifmt edif ^
-o "%VAR_NAME%.jed" ^
-lib "%VAR_FITTERDIR%\aprim.lib" ^
-tech "%VAR_DEVICE%" ^
-device "%VAR_PACKAGE%" ^
-tpd "%VAR_SPEED%"

%VAR_FITTERDIR%\%VAR_FITTER% ^
-i "%VAR_NAME%.edif" ^
-ifmt edif ^
-o "%VAR_NAME%.jed" ^
-lib "%VAR_FITTERDIR%\aprim.lib" ^
-tech "%VAR_DEVICE%" ^
-device "%VAR_PACKAGE%" ^
-tpd "%VAR_SPEED%"

findstr /R /C:"^    " "%VAR_NAME%.fit"
echo.

echo Logic Array Block       Macro Cells     I/O Pins        Foldbacks       TotalPT         FanIN           Cascades
findstr /R /C:"^[A-H]:" "%VAR_NAME%.fit"
echo.

findstr /R /C:"^Total" "%VAR_NAME%.fit"
echo.

findstr /R /C:"^$Device" "%VAR_NAME%.fit"
echo.

exit /b 0
