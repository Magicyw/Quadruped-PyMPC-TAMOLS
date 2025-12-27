"""Video recording module for MuJoCo simulations.

This module provides functionality to record videos from MuJoCo simulations
by capturing frames using offscreen rendering and saving them as MP4 files.

Based on working MuJoCo recording patterns from the community.
"""

import time
from datetime import datetime
from pathlib import Path

import mujoco
import numpy as np


class VideoRecorder:
    """Records video from MuJoCo simulations using offscreen rendering.
    
    This implementation follows the proven pattern:
    - Creates mujoco.Renderer with moderate resolution (640x480 default)
    - Updates renderer with simulation data each frame
    - Captures frames with simple renderer.render() call
    - Saves using imageio/mediapy
    
    Attributes:
        viewer: MuJoCo viewer handle
        model: MuJoCo model (mjModel)
        data: MuJoCo data (mjData)
        output_dir: Directory to save recordings
        width: Video width in pixels
        height: Video height in pixels
        fps: Frames per second for video
        is_recording: Whether currently recording
        frames: List of captured frames
        renderer: MuJoCo Renderer for offscreen rendering
    """

    def __init__(
        self,
        viewer,
        model,
        data,
        output_dir: str = "recordings",
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ):
        """Initialize the video recorder.
        
        Args:
            viewer: MuJoCo viewer handle
            model: MuJoCo model (mjModel)
            data: MuJoCo data (mjData)
            output_dir: Directory to save recordings
            width: Video width in pixels (default 640, moderate resolution)
            height: Video height in pixels (default 480, moderate resolution)
            fps: Frames per second for video
        """
        self.viewer = viewer
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
        
        # Get model and data from viewer if it's a Handle type
        if hasattr(viewer, 'm') and hasattr(viewer, 'd'):
            # MuJoCo passive viewer Handle
            self.model = viewer.m
            self.data = viewer.d
            print("📹 VideoRecorder: Using viewer's model and data (Handle)")
        else:
            # Use provided model and data
            self.model = model
            self.data = data
            print("📹 VideoRecorder: Using provided model and data")
        
        # Create offscreen renderer with moderate resolution
        # Following the working pattern: mujoco.Renderer(model, height, width)
        print(f"🔧 Creating mujoco.Renderer({self.width}x{self.height})...")
        
        try:
            # Simple renderer creation - no complex parameters needed
            self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
            print(f"✅ VideoRecorder initialized successfully!")
            print(f"   Resolution: {self.width}x{self.height} @ {self.fps} FPS")
            print(f"   Output directory: {self.output_dir.absolute()}")
            print(f"   Press 'V' to start/stop recording")
        except Exception as e:
            print(f"❌ Failed to create mujoco.Renderer: {e}")
            print(f"   Error type: {type(e).__name__}")
            self.renderer = None
            print(f"   ⚠️  Video recording is DISABLED")

    def toggle_recording(self):
        """Toggle recording state (start/stop)."""
        current_time = time.time()
        
        # Debounce key press
        if current_time - self.last_key_press_time < self.key_debounce_time:
            return
        
        self.last_key_press_time = current_time
        
        if not self.is_recording:
            # Start recording
            if self.renderer is None:
                print("❌ Cannot start recording: Renderer not available")
                print("   Check initialization errors above for details.")
                return
            self.is_recording = True
            self.frames = []
            print("🎥 Recording started...")
        else:
            # Stop recording and save
            self.is_recording = False
            print("⏹️  Recording stopped. Saving video...")
            self.save_video()

    def capture_frame(self):
        """Capture the current frame if recording is active.
        
        Uses the simple pattern from working examples:
        - Update renderer with current data
        - Call renderer.render() to get pixels
        - Store the frame
        """
        if not self.is_recording or self.renderer is None:
            return
        
        try:
            # Update renderer with current simulation data
            # Following the working pattern: renderer.update_scene(data)
            self.renderer.update_scene(self.data)
            
            # Render and capture pixels - simple and direct
            frame = self.renderer.render()
            
            if frame is not None and frame.size > 0:
                self.frames.append(frame.copy())
            else:
                if len(self.frames) == 0:
                    print(f"⚠️  First frame is empty")
                
        except Exception as e:
            if len(self.frames) == 0:
                print(f"❌ Frame capture failed: {e}")
                import traceback
                traceback.print_exc()

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
            print(f"💾 Saving {len(self.frames)} frames to {output_path.name}...")
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
            import traceback
            traceback.print_exc()
        
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
