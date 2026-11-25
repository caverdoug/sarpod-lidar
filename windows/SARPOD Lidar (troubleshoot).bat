@echo off
rem Same window, but keeps a console open so any error message stays on screen.
"%~dp0python\python.exe" "%~dp0app\sarpod_lidar_gui.pyw" %*
pause
