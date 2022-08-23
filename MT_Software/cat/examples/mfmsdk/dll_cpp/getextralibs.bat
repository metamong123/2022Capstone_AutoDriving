@echo off

if "%1"=="" (
	call:copystuff x64 64
	if errorlevel 1 exit /B
	call:copystuff win32 32
) else (
	call:copystuff %1 %2
)
exit /B


:copystuff
	setlocal
	set platform=%1
	set bits=%2
	(robocopy ..\..\3p\intel\win%bits%\bin\ ..\..\%platform%\lib libifcoremd.dll libmmd.dll /NJH /NJS /NP) ^& IF errorlevel 8 (exit /B) ELSE (cd . >nul)
	(robocopy ..\..\3p\lapack\win%bits%\bin\ ..\..\%platform%\lib libiomp5md.dll /NJH /NJS /NP) ^& IF errorlevel 8 (exit /B) ELSE (cd . >nul)
