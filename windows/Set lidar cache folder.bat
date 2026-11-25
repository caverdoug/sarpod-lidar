@echo off
rem Choose where lidar tiles are kept on this computer (e.g. a Google Drive folder shared by all your computers).
"%~dp0python\python.exe" "%~dp0app\lidar_sweep_map.py" --set-cache
