"""Video recording module for MuJoCo simulations.

This module provides functionality to record videos from MuJoCo simulations
by capturing frames using offscreen rendering and saving them as MP4 files.
"""

import os
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np


class VideoRecorder:
    """Records video from MuJoCo viewer using offscreen rendering.
    
    Attributes:
        viewer: MuJoCo viewer instance
        model: MuJoCo model
        data: MuJoCo data
        output_dir: Directory to save recordings
        width: Video width in pixels
        height: Video height in pixels
        fps: Frames per second for video
        is_recording: Whether currently recording
        frames: List of captured frames
    """

    def __init__(
        self,
        viewer,
        model,
        data,
        output_dir: str = "recordings",
        width: int = 1920,
        height: int = 1080,
        fps: int = 30,
    ):
        """Initialize the video recorder.
        
        Args:
            viewer: MuJoCo viewer instance
            model: MuJoCo model (mjModel)
            data: MuJoCo data (mjData)
            output_dir: Directory to save recordings
            width: Video width in pixels
            height: Video height in pixels
            fps: Frames per second for video
        """
        self.viewer = viewer
        self.model = model
        self.data = data
        self.output_dir = Path(output_dir)
        self.width = width
        self.height = height
        self.fps = fps
        
        self.is_recording = False
        self.frames = []
        self.last_key_press_time = 0
        self.key_debounce_time = 0.5  # seconds
        
        # Create output directory if it doesn't exist
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Create offscreen rendering context
        self._setup_offscreen_rendering()
        
        print(f"VideoRecorder initialized. Output directory: {self.output_dir.absolute()}")
        print("Press 'V' to start/stop recording")

    def _setup_offscreen_rendering(self):
        """Set up offscreen rendering context for capturing frames."""
        # Create an offscreen OpenGL context
        self.offscreen_context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        
        # Create a viewport for offscreen rendering
        self.viewport = mujoco.MjrRect(0, 0, self.width, self.height)
        
        # Create a scene for rendering
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        
        # Create camera
        self.camera = mujoco.MjvCamera()
        
        # Create rendering options (copy from viewer if available)
        self.render_options = mujoco.MjvOption()

    def toggle_recording(self):
        """Toggle recording state (start/stop)."""
        current_time = time.time()
        
        # Debounce key press
        if current_time - self.last_key_press_time < self.key_debounce_time:
            return
        
        self.last_key_press_time = current_time
        
        if not self.is_recording:
            # Start recording
            self.is_recording = True
            self.frames = []
            print("🎥 Recording started...")
        else:
            # Stop recording and save
            self.is_recording = False
            print("⏹️  Recording stopped. Saving video...")
            self.save_video()

    def capture_frame(self):
        """Capture the current frame if recording is active."""
        if not self.is_recording:
            return
        
        # Copy camera settings from viewer using MuJoCo's copy function
        if hasattr(self.viewer, 'cam'):
            # Use MuJoCo's camera copy function to avoid reference issues
            mujoco.mjv_copyCamera(self.model, self.camera, self.viewer.cam)
        
        # Update scene
        mujoco.mjv_updateScene(
            self.model,
            self.data,
            self.render_options,
            None,  # No perturbation
            self.camera,
            mujoco.mjtCatBit.mjCAT_ALL,
            self.scene,
        )
        
        # Render the scene to offscreen buffer
        mujoco.mjr_render(self.viewport, self.scene, self.offscreen_context)
        
        # Read pixels from the offscreen buffer
        rgb_array = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(rgb_array, None, self.viewport, self.offscreen_context)
        
        # Flip image vertically (MuJoCo uses OpenGL convention with origin at bottom-left)
        rgb_array = np.flipud(rgb_array)
        
        # Store the frame
        self.frames.append(rgb_array)

    def save_video(self):
        """Save the recorded frames to an MP4 file."""
        if not self.frames:
            print("⚠️  No frames to save")
            return
        
        try:
            import imageio
        except ImportError:
            print("❌ imageio not installed. Cannot save video.")
            print("   Install with: pip install imageio imageio-ffmpeg")
            return
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = self.output_dir / f"recording_{timestamp}.mp4"
        
        # Save video using imageio
        try:
            imageio.mimsave(
                output_path,
                self.frames,
                fps=self.fps,
                codec='libx264',
                quality=8,  # High quality (scale 1-10)
                pixelformat='yuv420p',
            )
            print(f"✅ Video saved: {output_path.absolute()}")
            print(f"   Frames: {len(self.frames)}, Duration: {len(self.frames)/self.fps:.2f}s")
        except Exception as e:
            print(f"❌ Error saving video: {e}")
        
        # Clear frames to free memory
        self.frames.clear()

    def cleanup(self):
        """Cleanup and save any remaining recording."""
        if self.is_recording:
            print("\n🎬 Finalizing recording...")
            self.is_recording = False
            self.save_video()
        
        # Free offscreen context
        if hasattr(self, 'offscreen_context'):
            del self.offscreen_context
        if hasattr(self, 'scene'):
            del self.scene
