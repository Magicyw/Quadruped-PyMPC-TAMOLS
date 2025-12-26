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
        
        Attempts multiple methods to capture frames from the simulation.
        """
        self.renderer = None
        self.capture_method = None
        
        # Method 1: Try to create an independent Renderer
        try:
            self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
            self.capture_method = "renderer"
            print("✓ Using mujoco.Renderer for offscreen rendering")
            return
        except Exception as e:
            print(f"⚠️  Could not create mujoco.Renderer: {e}")
        
        # Method 2: For MuJoCo passive viewer Handle, use viewer's model
        # The Handle object has .m (model) and .d (data) attributes
        if hasattr(self.viewer, 'm') and hasattr(self.viewer, 'd'):
            try:
                # Try to create renderer with viewer's model
                self.renderer = mujoco.Renderer(self.viewer.m, height=self.height, width=self.width)
                self.capture_method = "handle_renderer"
                print("✓ Using mujoco.Renderer with viewer's model (Handle)")
                return
            except Exception as e:
                print(f"⚠️  Could not create Renderer with viewer's model: {e}")
        
        # Method 3: Check if viewer has a context we can use
        if hasattr(self.viewer, '_render_context'):
            self.capture_method = "viewer_context"
            print("✓ Using viewer's render context for frame capture")
            return
            
        # Method 4: Try to capture directly from viewer's framebuffer
        if hasattr(self.viewer, 'read_pixels'):
            self.capture_method = "viewer_pixels"
            print("✓ Using viewer's read_pixels for frame capture")
            return
        
        # Method 5: Use viewer's internal context (mujoco passive viewer)
        if hasattr(self.viewer, 'ctx') or hasattr(self.viewer, '_ctx'):
            self.capture_method = "viewer_ctx"
            print("✓ Using viewer's OpenGL context for frame capture")
            return
            
        print("⚠️  No suitable rendering method found. Will attempt to capture from viewer during recording.")

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
        
        try:
            rgb_array = None
            
            # Method 1: Use independent Renderer
            if self.capture_method == "renderer" and self.renderer is not None:
                if hasattr(self.viewer, 'cam'):
                    self.renderer.update_scene(self.data, camera=self.viewer.cam)
                else:
                    self.renderer.update_scene(self.data)
                rgb_array = self.renderer.render()
            
            # Method 2: Use Renderer with viewer's model (Handle type)
            elif self.capture_method == "handle_renderer" and self.renderer is not None:
                # For MuJoCo passive viewer Handle, use viewer's data and camera
                if hasattr(self.viewer, 'd') and hasattr(self.viewer, 'cam'):
                    self.renderer.update_scene(self.viewer.d, camera=self.viewer.cam)
                    rgb_array = self.renderer.render()
                elif hasattr(self.viewer, 'd'):
                    self.renderer.update_scene(self.viewer.d)
                    rgb_array = self.renderer.render()
            
            # Method 3: Use viewer's render context
            elif self.capture_method == "viewer_context":
                # Render using the viewer's context
                if hasattr(self.viewer, '_render_context'):
                    # This would need viewer-specific implementation
                    pass
            
            # Method 4: Read pixels from viewer
            elif self.capture_method == "viewer_pixels":
                if hasattr(self.viewer, 'read_pixels'):
                    rgb_array = self.viewer.read_pixels(depth=False)
                    if rgb_array is not None:
                        rgb_array = np.flipud(rgb_array)
            
            # Method 5: Use viewer's OpenGL context
            elif self.capture_method == "viewer_ctx":
                # Try to render using viewer's context
                ctx = getattr(self.viewer, 'ctx', None) or getattr(self.viewer, '_ctx', None)
                if ctx is not None:
                    # Use mjr_readPixels with viewer's context
                    try:
                        viewport = mujoco.MjrRect(0, 0, self.width, self.height)
                        rgb_array = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                        mujoco.mjr_readPixels(rgb_array, None, viewport, ctx)
                        rgb_array = np.flipud(rgb_array)
                    except Exception as e:
                        if len(self.frames) == 0:
                            print(f"⚠️  Frame capture with viewer context failed: {e}")
            
            # Fallback: Try any available method
            else:
                # Try read_pixels as last resort
                if hasattr(self.viewer, 'read_pixels'):
                    try:
                        rgb_array = self.viewer.read_pixels(depth=False)
                        if rgb_array is not None:
                            rgb_array = np.flipud(rgb_array)
                    except:
                        pass
                
                # If still no array, try to access viewer's internal framebuffer
                if rgb_array is None:
                    # For passive viewer, try to get the viewport pixels
                    if hasattr(self.viewer, '_get_viewer_pixels'):
                        rgb_array = self.viewer._get_viewer_pixels()
                    elif hasattr(self.viewer, 'viewport'):
                        # Try manual pixel reading
                        try:
                            import glfw
                            import OpenGL.GL as gl
                            if hasattr(self.viewer, 'window'):
                                glfw.make_context_current(self.viewer.window)
                                width, height = glfw.get_framebuffer_size(self.viewer.window)
                                rgb_array = gl.glReadPixels(0, 0, width, height, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)
                                rgb_array = np.frombuffer(rgb_array, dtype=np.uint8).reshape(height, width, 3)
                                rgb_array = np.flipud(rgb_array)
                                # Resize if needed
                                if width != self.width or height != self.height:
                                    try:
                                        from PIL import Image
                                        img = Image.fromarray(rgb_array)
                                        img = img.resize((self.width, self.height))
                                        rgb_array = np.array(img)
                                    except:
                                        pass  # Keep original size if resize fails
                        except Exception as e:
                            if len(self.frames) == 0:
                                print(f"⚠️  OpenGL pixel reading failed: {e}")
            
            # Validate and store frame
            if rgb_array is not None and rgb_array.size > 0:
                self.frames.append(rgb_array)
            elif len(self.frames) == 0:
                print(f"⚠️  No valid frame captured. Capture method: {self.capture_method}")
                print(f"   Viewer type: {type(self.viewer).__name__}")
                print(f"   Viewer attributes: {[attr for attr in dir(self.viewer) if not attr.startswith('_')][:10]}")
                
        except Exception as e:
            if len(self.frames) == 0:
                print(f"⚠️  Frame capture error: {e}")
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
