@echo off
rem Understory and line-of-sight map from leaf-off lidar around a marker in a SARTopo export.
rem Double-click (it asks for the export, marker and radius), or drag a SARTopo .json onto this file.
"%~dp0python\python.exe" "%~dp0app\lidar_sweep_map.py" %*
pause
