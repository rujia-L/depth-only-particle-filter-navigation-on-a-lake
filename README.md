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
```

- `X_t, Y_t`: 2D position on the map  
- `Phi_t`: heading angle

### Observation (depth-only)
At each step, the only measurement is depth:

```text
Z_t = h(X_t, Y_t) * (1 + eps_t)
eps_t ~ Normal(0, sigma_eps^2)
```

- `h(X, Y)`: depth map lookup at position `(X, Y)`
- `eps_t`: multiplicative sensor noise (depth gauge uncertainty)

### Motion model (control + drift/noise)
With commanded forward displacement `d_t` and commanded turn `delta_phi_t`:

```text
X_t   = X_{t-1} + (d_t + eta_d) * cos(Phi_{t-1} + delta_phi_t + eta_phi)
Y_t   = Y_{t-1} + (d_t + eta_d) * sin(Phi_{t-1} + delta_phi_t + eta_phi)
Phi_t = Phi_{t-1} + delta_phi_t + eta_phi
```

- `eta_d`: translational noise (wind/current/actuation)  
- `eta_phi`: rotational noise  

**No compass is required**: heading is inferred indirectly from motion + depth consistency.

---

## Method

### Particle Filter
Maintain particles `{x_t^(i), w_t^(i)}` representing pose hypotheses.

At each time step:

```text
1) Predict: propagate each particle through the motion model
2) Update: compute weight w_t^(i) proportional to p(Z_t | x_t^(i))
3) Normalize weights
4) Resample if degeneracy is high (low ESS)
```

### Feasibility and collision handling
The map defines a valid water domain. If a proposed particle move crosses into land/outside the domain:

```text
- keep position unchanged (no forward progress)
- apply a large/randomized heading change ("bounce")
```

This prevents particles from accumulating in invalid regions and approximates physical grounding near shorelines.

### Resampling rule (ESS)
Use effective sample size (ESS) to trigger resampling:

```text
ESS = 1 / sum_i (w_i^2)
resample if ESS < threshold
```

---

## Control Policy (Explore → Go Home)
A simple two-phase controller:

```text
Phase 1: Exploration
- execute structured motion (e.g., circle/arc) to collect informative depth readings

Phase 2: Navigation
- steer toward the known home location using the current state estimate
- keep filtering online while moving
```

---

## Experiments (high level)
- **Lake environments:** depth variation is informative → filter typically converges and navigation succeeds.
- **Ocean-scale environments:** smoother depth fields reduce identifiability → requires parameter scaling
  (more particles, longer exploration, larger motion scale) and may still fail in near-flat bathymetry.

---

## Repository Structure (suggested)

```text
src/
  particle_filter.*        # predict/update/resample
  motion_model.*           # dynamics + collision handling
  controller.*             # exploration + go-home policy
maps/                      # bathymetric maps + home location
experiments/               # lake/ocean configs
results/                   # trajectories, particle clouds, error curves
```

---

## How to Run
> Replace the paths/filenames below with your actual entry points.

```text
1) Select a map configuration (lake/ocean) and parameters:
   - N_particles, sigma_eps, motion noise, horizon T, switch step, docking radius

2) Run the simulation loop:
   initialize particles over valid water domain
   for t = 1..T:
     apply control
     predict particles
     update weights using depth likelihood
     resample if ESS low
     log estimates and plots

3) Save results under results/ (plots and summary metrics)
```

---

## Notes
- Depth-only measurements can create **multi-modal posteriors** (many map points share similar depth).
- Particle filtering is well-suited for this regime compared to unimodal filters (EKF/UKF).
- If you want stronger baselines, add an EKF/UKF and show failure cases under multi-modality.
