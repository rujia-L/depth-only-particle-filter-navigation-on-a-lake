# Odometry- and Wall-Aware Particle Filter Navigation (Depth-Only)

## What this script does
`Main_PF_Final_Odometry_Aware.m` runs a full simulation of **depth-only localization and navigation** on a known bathymetric map using a **particle filter**, with two key engineering fixes:

1. **Look-ahead collision detection (physics layer)**  
   Prevents the true boat from stepping onto land/shallow water. If a move would cross the shoreline, the boat stays in place and “bounces” by turning sharply.

2. **Odometry-aware heading update (particle layer)**  
   Particles do **not** use the commanded turn `turn_cmd`. Instead, they use the **actual realized turn** `odom_turn` (difference between the new and old true heading), mimicking an IMU/odometry reading.  
   This fixes the common failure mode where the true boat bounces/turns at a wall but particles keep following the command.

3. **Map-aware wall constraint for particles**  
   Each particle proposes a move. If the proposed position is on land/outside the map (invalid depth), the particle’s **position is rolled back** while keeping the updated heading.  
   This prevents particles from “walking through land”.

## Inputs
- `Oresund.mat` (or another `*.mat` map) must contain:
  - `Depth` (grid depth field)
  - `xscale`, `yscale` (coordinate axes)
  - `harbour` (home location), optional

## How to run
1. Put your map file (e.g., `Oresund.mat`) in the working directory.
2. In MATLAB, run:
   ```matlab
   run("Main_PF_Final_Odometry_Aware.m")
