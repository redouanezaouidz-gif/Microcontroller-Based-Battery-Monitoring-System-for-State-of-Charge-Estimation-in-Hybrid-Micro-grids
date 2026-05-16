# Microcontroller-Based-Battery-Monitoring-System-for-State-of-Charge-Estimation-in-Hybrid-Micro-grids
Thesis simulation and codes used 
# SoC Estimation using a Standard Kalman Filter (SKF)

This repository contains the MATLAB/Simulink simulation of a linear Kalman filter
for estimating the State of Charge (SoC) of a 12 V 7 Ah lead‑acid battery.

## Files
- `battery_soc_sim.slx`  – Simulink model (plant + measurement + estimator)
- `plot_results.m`       – Post‑processing script (if any)

## Requirements
- MATLAB & Simulink (any version with the Simulink toolbox)

## Real‑Time Kalman Filter on ESP32

The code ran on the ESP32 microcontroller (`battery_soc_wt32.ino`).
