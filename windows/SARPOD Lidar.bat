@echo off
rem Opens the SARPOD Lidar map window.  Drag a SARTopo .json onto this file to open it straight away.
start "" "%~dp0python\pythonw.exe" "%~dp0app\sarpod_lidar_gui.pyw" %*
