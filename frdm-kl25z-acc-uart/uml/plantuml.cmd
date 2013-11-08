@echo off
REM requires plantuml.jar to be in the path in order to work

for %%i in (plantuml.jar) do SET PLANTUML=%%~$PATH:i
java -jar %PLANTUML% -verbose %*