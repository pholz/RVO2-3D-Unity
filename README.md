RVO2-Unity
===

use rvo2 (Optimal Reciprocal Collision Avoidance) in unity. 

https://github.com/snape/RVO2-CS
https://github.com/snape/RVO2-3D

Adapted from:

https://github.com/Hengle/RVO2-3D-Unity

## Overview

There are 2 branches: `2.5d` and `3d`.

Both feature a test environment in which the user can spawn agents and assign them random targets with almost guaranteed path crossings.

## 2.5d Branch

This is a slight adaptation of the original RVO2-3D-Unity repository, with a different test environment and some other extensions.

It uses the basic RVO2 (2D) approach and applies it to Unity's `XZ` plane. Data structures used (vectors etc.) are 3D types, but agents only avoid each other in two dimensions. It also includes obstacle avoidance (polygons).

Suitable for e.g. robots moving on the ground.

## 3d Branch

This is a complete port of RVO2-3D to the Unity environment, i.e. agents avoid each other in 3D. Like in RVO2-3D, there is no obstacle avoidance.

Suitable for e.g. UAVs.

## Requirements

* Unity 2018.3.2f2

