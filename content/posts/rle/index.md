---
title: "Reinforcement Learning Exploration"
date: 2025-06-12T09:00:00+00:00
description: RL for mobile robot navigation
hero: images/jackal.png
author:
  image: /images/sharwin_portrait.jpg
menu:
  sidebar:
    name: RL Exploration
    identifier: rle
    weight: 5
tags:  ["Python", "Reinforcement Learning", "Mujoco", "PPO", "LiDAR"]
repo: https://github.com/HarrisonBounds/RLE
---
The Jackal is a simple differential drive robot with an impressive payload capacity and can be equipped with various sensors. In this project, we simulated the jackal with a 3D LiDaR (Velodyne VLP16) in Mujoco and setup a PPO pipeline to teach the mobile robot to navigate towards a goal and avoid obstacles.

<div style="position: relative; width: 100%; padding-top: 56.25%; margin: auto;">
  <iframe
    src="https://www.youtube.com/embed/ywLXkt1FUGY"
    style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
    allowfullscreen>
  </iframe>
</div>