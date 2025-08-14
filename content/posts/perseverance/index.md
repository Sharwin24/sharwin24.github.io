---
title: "Perseverance"
date: 2025-08-12T09:00:00+00:00
description: EKF-based state estimation and navigation for a Mars rover–inspired 6‑wheel robot
hero: images/kalman_filter_errors_comparison.png
author:
  image: /images/sharwin_portrait.jpg
menu:
  sidebar:
    name: Perseverance
    identifier: perseverance
    weight: 5
tags: ["Python", "C++", "Extended Kalman Filter", "Sensor Fusion", "Robot Dynamics"]
repo: https://github.com/Sharwin24/Perseverance
---
Perseverance is an open‑source mobile robot inspired by NASA’s Mars rover. The project focuses on robust state estimation for a 6‑wheel rocker‑bogie platform using an Extended Kalman Filter (EKF) within a modular ROS 2 package for sensor fusion and, later, SLAM and navigation. It derives the non‑linear kinematic model and Jacobians, implements multiple drive‑base models (differential drive, mecanum, and skid‑steer), and benchmarks performance using IMU and wheel‑odometry data.

<div align="center"><em>Work in progress — more results and equations coming soon.</em></div>

<div align="center" style="display: flex; justify-content: center; align-items: center; flex-wrap: wrap;">
  <img src="images/kalman_filter_states_comparison.png" alt="Kalman Filter States Comparison" style="border-radius: 15px; width: 45%; margin: 10px;">
  <img src="images/kalman_filter_errors_comparison.png" alt="Kalman Filter Errors Comparison" style="border-radius: 15px; width: 45%; margin: 10px;">
</div>
