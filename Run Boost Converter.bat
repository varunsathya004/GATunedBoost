@echo off
title MATLAB GATunedBoost Project Launcher
echo =======================================================
echo Starting MATLAB in Project Directory...
echo Pre-warming Parallel Pool via startup.m
echo =======================================================

:: Define your project path
set PROJECT_PATH=C:\Users\varun\OneDrive\Desktop\boostconverter_pid\GATunedBoost
set MODEL_NAME=boostconverter_pid.slx

:: Launch MATLAB in the project directory
:: It will automatically execute startup.m first, then open the model
matlab -nosplash -sd "%PROJECT_PATH%" -r "open('%MODEL_NAME%');"

echo MATLAB is initializing.
timeout /t 3 >nul
exit