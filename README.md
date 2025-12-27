<div style="display: flex; justify-content: space-around;">
  <img src="./gifs/aliengo_trot.gif" alt="Trot" width="30%">
  <img src="./gifs/hyqreal_pace.gif" alt="Pace" width="30%">
  <img src="./gifs/go2_bound.gif" alt="Bound" width="30%">
</div>


## Overview
This repo contains a model predictive controller based on the **single rigid body model** and written in **Python**. It comes in two flavours: gradient-based via [acados](https://github.com/acados/acados) or sampling-based via [jax](https://github.com/google/jax). The controller is tested on real robots and is compatible with [Mujoco](https://mujoco.org/). 


Features gradient-based mpc:
- less than 5ms computation on an intel i7-13700H cpu 
- optional integrators for model mismatch
- optional smoothing for the ground reaction forces 
- optional foothold optimization
- optional [real-time iteration](http://cse.lab.imtlucca.it/~bemporad/publications/papers/ijc_rtiltv.pdf) or [advanced-step real-time iteration](https://arxiv.org/pdf/2403.07101.pdf)
- optional zero moment point/center of mass constraints
- optional lyapunov-based criteria


Features sampling-based mpc:
- 10000 parallel rollouts in less than 2ms on an nvidia 4050 mobile gpu!
- optional step frequency adaptation for enhancing robustness
- implements different strategies: [random sampling](https://arxiv.org/pdf/2212.00541.pdf), [mppi](https://sites.gatech.edu/acds/mppi/), or [cemppi](https://arxiv.org/pdf/2203.16633.pdf) 
- different control parametrizations: zero-order, linear splines or cubic splines (see [mujoco-mpc](https://arxiv.org/pdf/2212.00541.pdf))

Features terrain-aware foothold adaptation:
- TAMOLS-inspired terrain-aware foothold planner for rough terrain
- local search with edge avoidance, roughness penalty, and kinematic constraints
- configurable cost weights and search parameters
- custom stepping stones terrain for testing (梅花桩地图)
- easy integration: just set `scene: 'stepping_stones_medium'` in config.py
- see [TAMOLS documentation](docs/TAMOLS_FOOTHOLD_ADAPTATION.md), [stepping stones terrain](docs/STEPPING_STONES_TERRAIN.md), and [usage guide](docs/USING_STEPPING_STONES.md) for details

Features video recording:
- press 'V' key to start/stop recording during simulation
- captures high-quality MP4 videos at 1920x1080 @ 30 FPS
- automatic timestamped filenames saved to `recordings/` directory
- mouse-controlled camera viewpoint with fixed perspective during recording
- see [video recording documentation](docs/VIDEO_RECORDING.md) for details

## Installation and Run

See [here](https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/main/README_install.md).

### Quick Start with Stepping Stones Terrain

```bash
# 1. Install stepping stones terrain (one-time setup)
cd simulation
python install_stepping_stones.py

# 2. Edit config.py to use stepping stones
# Set: simulation_params['scene'] = 'stepping_stones_medium'
# Set: simulation_params['visual_foothold_adaptation'] = 'tamols'

# 3. Run simulation
python simulation.py
```

**Important**: The installer copies terrain files to gym_quadruped's scene directory. This only needs to be done once.

See [usage guide](docs/USING_STEPPING_STONES.md) for detailed configuration and troubleshooting.

## Citing this work

If you find the work useful, please consider citing one of our works: 

#### [On the Benefits of GPU Sample-Based Stochastic Predictive Controllers for Legged Locomotion (IROS-2024)](https://arxiv.org/abs/2403.11383):
```
@INPROCEEDINGS{turrisi2024sampling,
  author={Turrisi, Giulio and Modugno, Valerio and Amatucci, Lorenzo and Kanoulas, Dimitrios and Semini, Claudio},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={On the Benefits of GPU Sample-Based Stochastic Predictive Controllers for Legged Locomotion}, 
  year={2024},
  pages={13757-13764},
  doi={10.1109/IROS58592.2024.10801698}}
```
#### [Adaptive Non-Linear Centroidal MPC With Stability Guarantees for Robust Locomotion of Legged Robots (RAL-2025)](https://arxiv.org/abs/2409.01144):
```
@ARTICLE{elobaid2025adaptivestablempc,
  author={Elobaid, Mohamed and Turrisi, Giulio and Rapetti, Lorenzo and Romualdi, Giulio and Dafarra, Stefano and Kawakami, Tomohiro and Chaki, Tomohiro and Yoshiike, Takahide and Semini, Claudio and Pucci, Daniele},
  journal={IEEE Robotics and Automation Letters}, 
  title={Adaptive Non-Linear Centroidal MPC With Stability Guarantees for Robust Locomotion of Legged Robots}, 
  year={2025},
  volume={10},
  number={3},
  pages={2806-2813},
  doi={10.1109/LRA.2025.3536296}}
```

## Maintainer

This repository is maintained by [Giulio Turrisi](https://github.com/giulioturrisi) and [Daniel Ordonez](https://github.com/Danfoa).
