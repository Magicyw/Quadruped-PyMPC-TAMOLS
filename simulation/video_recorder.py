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
        """Set up offscreen rendering context for capturing frames.
        
        Uses mujoco.Renderer for safe offscreen rendering without
        requiring manual OpenGL context management.
        """
        try:
            # Use the modern Renderer API (MuJoCo 3.x+)
            # This handles offscreen rendering safely without segfaults
            self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
            print("✓ Using mujoco.Renderer for offscreen rendering")
        except Exception as e:
            print(f"⚠️  Could not create mujoco.Renderer: {e}")
            print("   Video recording will capture from viewer window instead")
            self.renderer = None

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
        
        if self.renderer is not None:
            # Use the Renderer API (preferred method)
            # Update camera to match viewer's camera
            if hasattr(self.viewer, 'cam'):
                self.renderer.update_scene(self.data, camera=self.viewer.cam)
            else:
                self.renderer.update_scene(self.data)
            
            # Render and get pixels
            rgb_array = self.renderer.render()
            
            # Store the frame (already in correct orientation)
            self.frames.append(rgb_array)
        else:
            # Fallback: try to capture from viewer window
            # This requires the viewer to have a read_pixels method
            try:
                if hasattr(self.viewer, 'read_pixels'):
                    rgb_array = self.viewer.read_pixels(depth=False)
                    # Flip if needed
                    rgb_array = np.flipud(rgb_array)
                    self.frames.append(rgb_array)
                else:
                    # Cannot capture - silently skip
                    pass
            except Exception as e:
                # Silently skip if capture fails
                pass

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
        
        # Free renderer resources
        if hasattr(self, 'renderer') and self.renderer is not None:
            self.renderer.close()
            del self.renderer
