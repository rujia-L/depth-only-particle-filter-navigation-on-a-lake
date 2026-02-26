## Simulation & Particle Filter Parameters

| Category | Parameter | Value |
|---|---|---:|
| Simulation | Total time steps (`T`) | 2500 |
| Simulation | Exploration → navigation switch step | 500 |
| Simulation | Docking radius | 5.0 |
| Simulation | Snapshot interval | 250 |
| Simulation | Water threshold | -0.3 |
| Controller | Nominal cruising speed | 0.6 |
| Controller | Reduced speed near home | 0.3 |
| Controller | Maximum steering angle | 30° |
| Motion noise | Translational noise std. (`σ_xy,act`) | 0.05 |
| Motion noise | Rotational noise std. (`σ_φ,act`) | 1° |
| Measurement | Relative depth noise ratio | 0.10 |
| Particle filter | Number of particles (`N`) | 5000 |
| Particle filter | Propagation std. in position (`σ_xy`) | 0.20 |
| Particle filter | Propagation std. in heading (`σ_φ`) | 3° |
| Particle filter | Measurement error scaling | 0.20 |
| Particle filter | ESS resampling threshold | 0.5·N |
