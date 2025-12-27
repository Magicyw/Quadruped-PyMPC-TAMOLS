# Video Recording Feature

## Overview

The video recording feature allows you to capture MuJoCo simulations directly from the viewer and save them as MP4 video files. This is useful for documentation, debugging, and sharing results.

## Features

- **Toggle recording with V key**: Press 'V' to start/stop recording at any time
- **Fixed viewpoint**: The camera viewpoint is controlled manually via mouse during simulation
- **High-quality output**: Records at 1920x1080 resolution at 30 FPS by default
- **Automatic timestamping**: Videos are automatically named with timestamps
- **Memory management**: Frames are cleared after saving to prevent memory issues

## Installation

The video recording feature requires the following dependencies:

```bash
pip install imageio imageio-ffmpeg
```

These are included in the `pyproject.toml` dependencies and will be automatically installed when you install the package.

## Usage

### Basic Usage

1. **Start the simulation** with rendering enabled:
   ```bash
   python simulation/simulation.py
   ```

2. **Adjust the camera view** using your mouse:
   - Left-click and drag to rotate the camera
   - Right-click and drag to pan
   - Scroll to zoom in/out

3. **Start recording** by pressing the `V` key on your keyboard:
   ```
   🎥 Recording started...
   ```

4. **Stop recording** by pressing `V` again:
   ```
   ⏹️  Recording stopped. Saving video...
   ✅ Video saved: /path/to/recordings/recording_20231226_143022.mp4
      Frames: 450, Duration: 15.00s
   ```

### Output Location

All recordings are saved to the `recordings/` directory in the project root. The directory is automatically created if it doesn't exist.

Video files are named using the format: `recording_YYYYMMDD_HHMMSS.mp4`

Example:
- `recording_20231226_143022.mp4` - recorded on December 26, 2023 at 14:30:22

## Configuration

The `VideoRecorder` class can be configured with custom parameters:

```python
from simulation.video_recorder import VideoRecorder

video_recorder = VideoRecorder(
    viewer=env.viewer,
    model=env.mjModel,
    data=env.mjData,
    output_dir="recordings",    # Output directory
    width=1920,                 # Video width in pixels
    height=1080,                # Video height in pixels
    fps=30                      # Frames per second
)
```

## Technical Details

### Offscreen Rendering

The video recorder uses MuJoCo's modern `Renderer` API for safe offscreen rendering:

1. Creates a `mujoco.Renderer` instance with specified resolution
2. Updates the scene with current simulation data
3. Copies camera configuration from the main viewer
4. Renders the scene to an offscreen buffer
5. Retrieves RGB pixels in the correct orientation

This approach avoids segmentation faults that can occur with manual OpenGL context management.

### Keyboard Input

The video recording is controlled via a separate keyboard listener thread:
- Runs in the background without blocking the simulation
- Uses the `readchar` library for keyboard input
- Automatically stops when the simulation ends

### Video Encoding

Videos are encoded using the following settings:
- Codec: `libx264` (H.264)
- Quality: 8/10 (high quality)
- Pixel format: `yuv420p` (compatible with most players)

## Troubleshooting

### "readchar not available"

If you see this warning, install readchar:
```bash
pip install readchar
```

### "imageio not installed"

If recording fails with this error, install imageio:
```bash
pip install imageio imageio-ffmpeg
```

### Low frame rate in recordings

The recording captures frames at the same rate as the renderer (30 Hz by default). If your simulation runs slower than real-time, the video will also play slower. This is normal and reflects the actual simulation speed.

### Memory issues with long recordings

For very long recordings (>5 minutes), consider stopping and restarting the recording periodically to free up memory. Each frame takes approximately 6 MB of RAM (1920x1080x3 bytes).

## Examples

### Recording a gait sequence

1. Start simulation: `python simulation/simulation.py`
2. Adjust camera to side view
3. Press `V` to start recording
4. Let the robot walk for 10-15 seconds
5. Press `V` to stop and save

### Recording multiple takes

You can start and stop recording multiple times during a single simulation run. Each recording will be saved as a separate file with a unique timestamp.

## Performance Impact

The video recording uses offscreen rendering and should have minimal impact on simulation performance:
- Offscreen rendering: ~1-2ms per frame
- Frame storage: negligible
- Video encoding: happens after recording stops

The simulation speed should remain at or near real-time even while recording.

## See Also

- [MuJoCo Visualization Documentation](https://mujoco.readthedocs.io/en/stable/programming/visualization.html)
- [imageio Documentation](https://imageio.readthedocs.io/)
