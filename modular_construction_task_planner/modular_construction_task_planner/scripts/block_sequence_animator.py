import numpy as np
import trimesh
import time
import imageio

from typing import List, Dict, Tuple

class TrimeshBlockAnimator:
    def __init__(self, init_data: Dict[str, dict], goal_data: Dict[str, dict], 
                 placement_sequence: List[str], color_array: List[List[float]]):
        """
            init_data: dict of initial block states
            goal_data: dict of target goal block states
            placement_sequence: List of movement tuples -> [ (block_name, start_xyz, end_xyz), ... ]
            color_array: list of RGB floats [0.0, 1.0] used for index fallbacks
        """
        self.init_data = init_data
        self.goal_data = goal_data
        self.sequence = placement_sequence
        self.color_array = color_array
        self.scene = trimesh.Scene()
        self.begin = False

        # State counters for the OpenGL callback loop
        self.current_step_idx = 0
        self.interpolation_alpha = 0.0
        self.animation_speed = 0.05  # Steps size per render frame iteration

        self._build_scene()

    def _to_trimesh_color(self, rgb_list: List[float], alpha: int = 255) -> List[int]:
        """Converts raw float [0.0, 1.0] RGB arrays into integer [0, 255] RGBA format"""
        return [int(c * 255) for c in rgb_list[:3]] + [alpha]

    def _build_scene(self):
        # Semi-transparent gray configuration for target footprint placement markers
        gray_goal_color = [180, 180, 180, 100]

        for idx, (name, init_properties) in enumerate(self.init_data.items()):
            if name == 'robot':
                continue
            size = init_properties['size']
            start_pos = init_properties['position']
            
            # Extract color properties directly from dict, fallback to index array if empty
            if 'color' in init_properties and init_properties['color'] is not None:
                raw_color = init_properties['color']
            else:
                raw_color = self.color_array[idx % len(self.color_array)]
            
            # 1. Generate Static Goal Reference Mesh
            if name in self.goal_data:
                goal_pos = self.goal_data[name]['position']
                goal_mesh = trimesh.creation.box(extents=size)
                goal_mesh.visual.face_colors = gray_goal_color # type: ignore
                goal_matrix = trimesh.transformations.translation_matrix(goal_pos)
                self.scene.add_geometry(goal_mesh, node_name=f"{name}_goal", transform=goal_matrix)

            # 2. Generate Active Assembly Block Mesh 
            active_mesh = trimesh.creation.box(extents=size)
            active_mesh.visual.face_colors = self._to_trimesh_color(raw_color, alpha=255) # type: ignore
            active_matrix = trimesh.transformations.translation_matrix(start_pos)
            self.scene.add_geometry(active_mesh, node_name=f"{name}_active", transform=active_matrix)

    def animation_callback(self, scene):
        if not self.begin:
            # time.sleep(5.0)
            self.begin = True

        if self.current_step_idx >= len(self.sequence):
            return  # Run complete

        block_name = self.sequence[self.current_step_idx]
        start_xyz, end_xyz = self.init_data[block_name]['position'], self.goal_data[block_name]['position']

        # Process step positions using standard float LERP updates
        current_xyz = (1.0 - self.interpolation_alpha) * np.array(start_xyz) + (self.interpolation_alpha * np.array(end_xyz))
        transform_matrix = trimesh.transformations.translation_matrix(current_xyz)

        # Update node matrix within scene runtime graph map
        scene.graph.update(f"{block_name}_active", matrix=transform_matrix)

        self.interpolation_alpha += self.animation_speed

        if self.interpolation_alpha >= 1.0:
            # Snap piece to correct position baseline coordinates and clear tracking indices
            scene.graph.update(f"{block_name}_active", matrix=trimesh.transformations.translation_matrix(end_xyz))
            self.interpolation_alpha = 0.0
            self.current_step_idx += 1
            # time.sleep(0.15) 

    def save_to_video(self, output_path: str = "assembly_output.mp4", 
                      steps_per_sequence: int = 30, pause_frames: int = 10, 
                      fps: int = 30, resolution: tuple = (1280, 720)):
        """
            Renders the active assembly layout directly into an MP4 video file.
            Requires installing: pip install imageio[ffmpeg]
        """
        print(f"Recording animation to {output_path}...")
        self._setup_profile_camera()
        # Continue with your existing video writer stream...
        writer = imageio.get_writer(output_path, fps=fps, codec='libx264', quality=9)

        initial_frame_bytes = self.scene.save_image(resolution=resolution, visible=False)
        writer.append_data(imageio.v3.imread(initial_frame_bytes))

        # 1. Force the Camera view angle parallel to the XY plane (0 elevation)
        # We look straight down the Y axis from a slight X angle offset for dimensional depth
        all_positions = [data['position'] for data in self.init_data.values()] + \
                        [data['position'] for data in self.goal_data.values()]
        center_of_mass = np.mean(all_positions, axis=0)

        # elev=0 makes it perfectly level with the floor/XY plane. azim=-45 adds angular perspective.
        self.scene.set_camera(angles=[np.radians(0), 0, np.radians(-45)], 
                              distance=12.0, 
                              center=center_of_mass)

        # 2. Spin up the background video stream pipe
        writer = imageio.get_writer(output_path, fps=fps, codec='libx264', quality=9)

        # Capture the initial environment baseline configuration frame
        initial_frame_bytes = self.scene.save_image(resolution=resolution, visible=False)
        writer.append_data(imageio.v3.imread(initial_frame_bytes))

        # 3. Step through the sequence path states
        for step_idx, block_name in enumerate(self.sequence):
            start_xyz = np.array(self.init_data[block_name]['position'])
            end_xyz = np.array(self.goal_data[block_name]['position'])

            # Animate the frame transformations mapping track
            for frame in range(steps_per_sequence):
                alpha = frame / float(steps_per_sequence - 1)
                current_xyz = (1.0 - alpha) * start_xyz + (alpha * end_xyz)

                # Update node transformations map inside graph context reference
                matrix = trimesh.transformations.translation_matrix(current_xyz)
                self.scene.graph.update(f"{block_name}_active", matrix=matrix)
                
                # Dump the raw back-buffer frame straight to image array 
                frame_bytes = self.scene.save_image(resolution=resolution, visible=False)
                frame_image = imageio.v3.imread(frame_bytes)
                writer.append_data(frame_image)

            # Append static hold frames so blocks don't tele-transport instantly
            for _ in range(pause_frames):
                writer.append_data(frame_image)

            print(f" -> Rendered sequence block {step_idx + 1}/{len(self.sequence)} ({block_name})")

        writer.close()
        print(f"Successfully exported video file to: {output_path}")

    def _setup_profile_camera(self):
        """Forces the camera to look at the structure perfectly horizontally from the side."""
        # 1. Calculate the spatial center of your block layout
        all_positions = [data['position'] for data in self.init_data.values()] + \
                        [data['position'] for data in self.goal_data.values()]
        center_of_mass = np.mean(all_positions, axis=0)

        # 2. FORCE PERFECT HORIZONTAL LEVEL PROFILE VIEW
        # angles=[pitch_radians, roll_radians, yaw_radians]
        # Pitch = 90 degrees puts the camera lens level with the horizon, looking straight across the XY plane.
        # Yaw = -45 degrees turns the camera horizontally so you see both the front and side faces of the structure.
        self.scene.set_camera(angles=[np.radians(90), 0, np.radians(-45)], 
                              distance=10.0, 
                              center=center_of_mass)

    def run(self):
        # Open live renderer canvas
        self.scene.show(callback=self.animation_callback)

# ==========================================
# Instantiation Test
# ==========================================
if __name__ == "__main__":
    test_init = {
        'block12': {
            'type': "dynamic",
            'position': [-3.0, 1.5, 0.5],
            'orientation': [0.0, 0.0, 0.0],
            'color': [0.6, 0.3, 0.7],
            'size': [1.0, 3.0, 1.0]
        }
    }

    test_goal = {
        'block12': {
            'type': "dynamic",
            'position': [0.0, 0.0, 0.5],
            'orientation': [0.0, 0.0, 0.0],
            'size': [1.0, 3.0, 1.0]
        }
    }

    test_seq = [
        'block12'
    ]

    fallback_colors = [[1.0, 0.0, 0.0]]

    animator = TrimeshBlockAnimator(test_init, test_goal, test_seq, fallback_colors)
    animator.run()