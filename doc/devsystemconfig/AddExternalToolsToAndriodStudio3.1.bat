echo off
if not exist "%USERPROFILE%\.AndroidStudio3.1\" (
	echo The destination directory %USERPROFILE%\.AndriodStudio3.1 does not exist.
	echo Is Andriod Studio version 3.1 installed?
	pause
	echo on
	exit /b
)
set targetFolder="%USERPROFILE%\.AndroidStudio3.1\config\tools\"
set targetFile="%targetFolder%External Tools.xml"
if not exist %targetFile% (
	goto copyFileToTarget
)
echo The file %targetFile% already exists.  Not copying.
set /p UserResponse="Do you want to see the contents of the existing file (Y/N)?"
if /I "%UserResponse%"=="Y" ( 
	type %targetFile%
	exit /b
)
echo on
exit /b

:copyFileToTarget

xcopy /S /F ".\External Tools.xml" %targetFolder%

pause
echo on
