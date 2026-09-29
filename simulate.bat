@echo off
cd /d "%~dp0"
set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk
set PATH=%JAVA_HOME%\bin;%PATH%
gradlew.bat simulateJava
pause