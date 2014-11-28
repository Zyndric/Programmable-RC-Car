rem Call this batch script in a 32-bit JRE when on Windows! 64-bit JRE won't load the 32-bit rxtx serial library.
rem We won't getting 64-bit rxtx binaries any time soon, since the project seems dead (sorry if not).

set basedir=%~dp0
set rxtxdir=%basedir%\lib\rxtx-2.1-7-bins-r2

set path=%PATH%;%rxtxdir%\Windows\i368-mingw32\

java -cp .;"%rxtxdir%\RXTXcomm.jar" -jar "%basedir%\dist\Programmable_RC_Controller.jar"
