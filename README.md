# Depth-Only Particle Filter Navigation on Bathymetric Maps

## Overview
This project addresses **navigation and self-localization with extremely limited sensing**:
a boat drifts to an unknown position with near-zero visibility and only has a **noisy depth gauge**.  
Given a known **bathymetric depth map**, the system must (1) infer the boat’s pose and (2) steer back to a known home location.

We formulate this as a **sequential Bayesian state-estimation** problem and solve it with a **particle filter** that tracks a multimodal posterior over position and heading under strong map nonlinearity and hard feasibility constraints. :contentReference[oaicite:2]{index=2}

## Problem Setup
### State
Hidden state at time *t*:
\[
x_t = (X_t, Y_t, \Phi_t)
\]
where \((X_t, Y_t)\) is 2D position and \(\Phi_t\) is heading. :contentReference[oaicite:3]{index=3}

### Observation (depth-only)
A scalar depth measurement:
\[
Z_t = h(X_t, Y_t)\,(1+\varepsilon_t),
\quad \varepsilon_t \sim \mathcal{N}(0,\sigma_\varepsilon^2)
\]
Depth sensor is low quality (±15%); we use a slightly conservative setting with \(\sigma_\varepsilon \approx 0.1\). :contentReference[oaicite:4]{index=4}

### Motion model (control + drift/noise)
With commanded forward displacement \(d_t\) and turn \(\Delta\phi_t\),
\[
\begin{aligned}
X_t &= X_{t-1} + (d_t+\eta_t^{(d)})\cos(\Phi_{t-1}+\Delta\phi_t+\eta_t^{(\phi)})\\
Y_t &= Y_{t-1} + (d_t+\eta_t^{(d)})\sin(\Phi_{t-1}+\Delta\phi_t+\eta_t^{(\phi)})\\
\Phi_t &= \Phi_{t-1} + \Delta\phi_t + \eta_t^{(\phi)}
\end{aligned}
\]
with Gaussian translational/rotational noise capturing wind, currents, and actuation uncertainty. :contentReference[oaicite:5]{index=5}

**No compass is used**: heading must be inferred indirectly from motion + depth consistency. :contentReference[oaicite:6]{index=6}

## Method
### Particle Filter
We maintain particles \(\{x_t^{(i)}, w_t^{(i)}\}_{i=1}^N\) representing hypotheses of pose.
Depth-only measurements often yield **multi-modal** location hypotheses early on, which makes particle filtering a natural fit. :contentReference[oaicite:7]{index=7}

Key design components:
- **Feasibility / shoreline constraint:** states on land/outside the map are invalid. Proposed moves into infeasible regions trigger a **collision handling** rule: position stays, heading “bounces” (large random-like change), reflecting loss of forward progress near shorelines. :contentReference[oaicite:8]{index=8}
- **Odometry-aware propagation:** during collision events, realized turning can deviate from commanded turning; we propagate particles using the realized orientation change to reduce systematic mismatch. :contentReference[oaicite:9]{index=9}
- **Resampling:** monitor weight degeneracy via **effective sample size (ESS)** and resample when ESS drops below a threshold; optionally reinitialize if likelihood collapses. :contentReference[oaicite:10]{index=10}

### Controller (explore → go-home)
A simple two-phase policy:
1. **Exploration phase:** execute a circular/structured motion to collect informative depth measurements without requiring localization.
2. **Navigation phase:** steer toward the known home location using the current state estimate, while filtering continues online. :contentReference[oaicite:11]{index=11}

## Experiments
We evaluate on multiple environments:
- **Lakes:** informative depth variation typically enables fast convergence and successful docking.
- **Ocean-scale maps:** smoother bathymetry reduces identifiability; the baseline (lake-tuned) configuration often fails unless parameters are rescaled. :contentReference[oaicite:12]{index=12}

### Example parameter settings (from experiments)
Typical lake configuration includes:
- horizon \(T=2500\), switch step ≈ 500, docking radius ≈ 5m
- particles \(N=5000\), PF propagation noise (pos/heading), ESS threshold ≈ 0.5N :contentReference[oaicite:13]{index=13}

For ocean-scale environments, successful runs required rescaling:
- longer horizon and exploration (e.g., \(T=3500\), switch ≈ 1000)
- more particles (e.g., \(N=20000\))
- larger exploratory motion scale (higher cruise speed) :contentReference[oaicite:14]{index=14}

## Key Takeaways
- **Depth-only localization works well** when the map has distinctive spatial depth structure (typical lake bathymetry).
- Failures often arise from: grounding/“stuck” dynamics near boundaries, repeated wall collisions, or **map symmetry** causing persistent multi-modality.
- In smooth ocean bathymetry, identifiability is weaker; **parameter scaling** (time horizon, particle count, exploration footprint) improves performance but cannot fully overcome lack of informative features. :contentReference[oaicite:15]{index=15}

## Repository Structure (suggested)
```text
src/
  particle_filter.*        # PF predict/update/resample
  motion_model.*           # dynamics + collision handling
  controller.*             # exploration + go-home policy
maps/                      # bathymetric maps + home location
experiments/               # lake/ocean configs
results/                   # figures (trajectories, particle clouds, error curves)
