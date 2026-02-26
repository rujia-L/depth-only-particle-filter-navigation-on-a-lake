# Depth-Only Particle Filter Navigation on Bathymetric Maps

## Overview
This project solves **self-localization and navigation with extremely limited sensing**: a boat has near-zero visibility and only a **noisy depth gauge**. Given a known **bathymetric (depth) map** and a fixed time budget (simulation steps), the goal is to (1) infer the boat’s pose and (2) steer back to a known home location.

The core method is a **particle filter** that maintains a multimodal posterior over the boat state under motion noise and hard feasibility constraints (shorelines / land).

---

## Problem Setup

### State
Hidden state at time `t`:

```text
x_t = (X_t, Y_t, Phi_t)
