# MATLAB .mat File Logging

## Overview

The simulation now supports logging all simulation data to MATLAB-compatible `.mat` files. This feature allows researchers to easily export and analyze simulation data using MATLAB or other tools that support the `.mat` format.

## Features

- **Complete Data Capture**: Records all state observations from `QuadrupedEnv.ALL_OBS` and all controller observations
- **Incremental Writing**: Data is written to file after each simulation step, preventing data loss if simulation crashes
- **Automatic Flattening**: Handles complex data structures (arrays, LegsAttr) by automatically flattening to 1D
- **Robust Handling**: Gracefully handles None/missing values by replacing with zeros
- **Descriptive Naming**: Creates informative filenames with robot, scene, parameters, seed, and timestamp
- **Organized Storage**: Files saved to `log/<scene_name>/` directory structure

## Data Format

Each `.mat` file contains two variables:

1. **`header`**: Array of column names (strings)
   - First column is always `"time"`
   - Remaining columns are observation names (e.g., `"base_pos_0"`, `"base_pos_1"`, `"base_pos_2"`)
   - Multi-dimensional observations are suffixed with their index

2. **`data`**: T × N matrix of float64 values
   - T = total number of simulation steps across all episodes
   - N = number of columns (time + all observations)
   - First column contains `env.simulation_time` for each step
   - Remaining columns contain the flattened observation values
   - **Updated continuously**: File is rewritten after each step, so partial data is available even if simulation is interrupted

## Usage

### Enable/Disable Logging

By default, `.mat` logging is **enabled**. To disable it:

```python
from quadruped_pympc import config as cfg

run_simulation(
    qpympc_cfg=cfg,
    mat_logging=False  # Disable .mat logging
)
```

### Output Location

Files are saved to: `log/<scene_name>/<filename>.mat`

Example filename:
```
go2_flat_linvel0.0-4.0_angvel-0.4-0.4_friction0.5-1.0_seed42_20260107_142530.mat
```

The filename includes:
- Robot name (e.g., `go2`)
- Scene name (e.g., `flat`)
- Linear velocity range (e.g., `linvel0.0-4.0`)
- Angular velocity range (e.g., `angvel-0.4-0.4`)
- Friction coefficient range (e.g., `friction0.5-1.0`)
- Random seed (e.g., `seed42`)
- Timestamp (e.g., `20260107_142530`)

### Loading in MATLAB

```matlab
% Load the data
data_struct = load('log/flat/go2_flat_linvel0.0-4.0_angvel-0.4-0.4_friction0.5-1.0_seed42_20260107_142530.mat');

% Access the data
header = data_struct.header;  % Column names
data = data_struct.data;      % Numerical data

% Extract specific columns
time = data(:, 1);  % Time column is always first

% Find column index by name
col_idx = find(strcmp(header, 'base_pos_0'));
base_pos_x = data(:, col_idx);

% Plot example
figure;
plot(time, base_pos_x);
xlabel('Time (s)');
ylabel('Base Position X (m)');
title('Robot Base Position Over Time');
```

### Loading in Python

```python
from scipy.io import loadmat
import numpy as np

# Load the data
mat_data = loadmat('log/flat/go2_flat_..._.mat')

header = mat_data['header'][0]  # Column names as array
data = mat_data['data']          # Numerical data as numpy array

# Extract specific columns
time = data[:, 0]  # Time is always first column

# Find column by name
col_idx = list(header).index('base_pos_0')
base_pos_x = data[:, col_idx]

# Plot
import matplotlib.pyplot as plt
plt.plot(time, base_pos_x)
plt.xlabel('Time (s)')
plt.ylabel('Base Position X (m)')
plt.show()
```

## Recorded Observations

### State Observations (from QuadrupedEnv)

All observations from `QuadrupedEnv.ALL_OBS` are recorded, which typically includes:
- Base position and velocity
- Base orientation and angular velocity
- Joint positions and velocities
- Feet positions and contact states
- And more...

The exact list depends on the `gym_quadruped` version.

### Controller Observations (from QuadrupedPyMPC)

The following controller observations are recorded:
- `ref_base_height`: Reference base height (scalar)
- `ref_base_angles`: Reference base angles (3D vector)
- `ref_feet_pos`: Reference feet positions (4 legs × 3D = 12 values)
- `nmpc_GRFs`: NMPC ground reaction forces (4 legs × 3D = 12 values)
- `nmpc_footholds`: NMPC foothold positions (4 legs × 3D = 12 values)
- `swing_time`: Swing phase timing for each leg (4 values)
- `phase_signal`: Gait phase signals (4 values)
- `lift_off_positions`: Lift-off positions for each leg (4 legs × 3D = 12 values)

## Dependencies

The feature requires `scipy` for `.mat` file export:

```bash
pip install scipy
```

Note: `scipy` is already included in the project's `pyproject.toml` dependencies.

## Error Handling

- If `scipy` is not installed, a clear error message is displayed and logging is disabled
- If data dimensions are inconsistent (which should never happen), an exception is raised to catch bugs
- None/missing values are automatically replaced with 0.0

## Performance Considerations

- Data is accumulated in memory and written to disk after each step
- File is rewritten completely at each step to ensure data persistence
- For very long simulations, this may cause I/O overhead
- Memory usage grows linearly with the number of steps
- The `write_every_n_steps` parameter in `MatLogger` can be adjusted to write less frequently (default: 1)
  - Example: `write_every_n_steps=10` writes only every 10 steps, reducing I/O

## File Size

Approximate file sizes:
- 1000 steps with 50 observations: ~400 KB
- 10000 steps with 50 observations: ~4 MB
- 100000 steps with 50 observations: ~40 MB

## Gitignore

The `log/` directory and `*.mat` files are automatically excluded from git commits via `.gitignore`.

## Troubleshooting

### "No module named 'scipy'"

Install scipy:
```bash
pip install scipy
```

### "Data dimension mismatch" error

This indicates a bug in the data collection logic. Please report this issue with:
- The error message
- Your simulation parameters
- Steps to reproduce

### Large file sizes

Reduce the number of simulation steps:
```python
run_simulation(
    qpympc_cfg=cfg,
    num_episodes=1,           # Reduce episodes
    num_seconds_per_episode=10  # Reduce duration
)
```

**Note**: With incremental writing, the file is continuously updated and may cause performance impact for very long simulations due to repeated file I/O. The `write_every_n_steps` parameter can be adjusted in the MatLogger initialization if needed.

### Cannot find .mat file

Check the console output for the exact path:
```
MATLAB .mat logging enabled. Will save to: /path/to/log/scene_name/filename.mat
```

The file is created after the first step and updated continuously throughout the simulation.

### Simulation interrupted

If the simulation is interrupted or crashes, the `.mat` file will contain all data up to the last completed step. This is a key advantage of incremental writing - you don't lose all data if something goes wrong.

## Example Analysis Script

Here's a complete example of loading and analyzing the data in Python:

```python
from scipy.io import loadmat
import numpy as np
import matplotlib.pyplot as plt

# Load data
mat_file = 'log/flat/your_file_name.mat'
mat_data = loadmat(mat_file)

header = mat_data['header'][0]
data = mat_data['data']

# Helper function to get column by name
def get_column(header, data, name):
    try:
        idx = list(header).index(name)
        return data[:, idx]
    except ValueError:
        print(f"Column '{name}' not found")
        return None

# Extract time and some example data
time = data[:, 0]
base_height = get_column(header, data, 'base_pos_2')

# Create plots
fig, axes = plt.subplots(2, 1, figsize=(10, 8))

# Plot base height
axes[0].plot(time, base_height)
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Height (m)')
axes[0].set_title('Base Height Over Time')
axes[0].grid(True)

# Plot GRF for one leg (if available)
grf_z = get_column(header, data, 'nmpc_GRFs_2')  # FL leg Z component
if grf_z is not None:
    axes[1].plot(time, grf_z)
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Force (N)')
    axes[1].set_title('Ground Reaction Force (FL leg, Z)')
    axes[1].grid(True)

plt.tight_layout()
plt.show()

# Print statistics
print(f"Simulation duration: {time[-1]:.2f} seconds")
print(f"Number of steps: {len(time)}")
print(f"Average step frequency: {len(time) / time[-1]:.1f} Hz")
print(f"Number of observations: {data.shape[1] - 1}")  # -1 for time column
```

## Integration with Existing Features

The `.mat` logging is designed to work alongside existing features:

- **H5Writer**: The original H5 recording still works when `recording_path` is provided
- **Video Recording**: Press 'V' during simulation to toggle video recording
- **Custom Scenes**: Works with all scene types including stepping stones

Both `.mat` and `.h5` files can be generated simultaneously.
