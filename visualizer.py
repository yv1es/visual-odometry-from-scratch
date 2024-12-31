from matplotlib.gridspec import GridSpec
import matplotlib.pyplot as plt
import numpy as np


class Visualizer:
    def __init__(self, range_threshold=3):
        self.range_threshold = range_threshold
        self.scaling = None

        self.keypoint_counts = []
        self.candidate_counts = []
        self.time_steps = []

        self.fig = plt.figure(figsize=(18, 10))
        # Create a grid with 2 rows and 2 columns:
        # Left column: Camera frame on top, Outlier (inliers) plot below.
        # Right column: Local trajectory spanning both rows.
        gs = GridSpec(2, 2, figure=self.fig, width_ratios=[1, 1], height_ratios=[1, 1])

        # Left column, top: Camera frame
        self.ax_img = self.fig.add_subplot(gs[0, 0])
        self.ax_img.set_title("Current Frame")
        self.ax_img.axis("off")  # Hide axes for image display

        # Left column, bottom: Keypoints and Candidate Count over time
        self.ax_inliers = self.fig.add_subplot(gs[1, 0])
        self.ax_inliers.set_title("Keypoints and Candidates Over Time")
        self.ax_inliers.set_xlabel("Time (steps)")
        self.ax_inliers.set_ylabel("Count")

        # Right column spanning both rows: Local Trajectory and Landmarks
        self.ax_local_trjectory = self.fig.add_subplot(gs[:, 1])
        self.ax_local_trjectory.set_title("Local Trajectory and Landmarks")
        self.ax_local_trjectory.set_xlabel("X")
        self.ax_local_trjectory.set_ylabel("Z")
        self.ax_local_trjectory.set_aspect("equal", adjustable="datalim")
        self.ax_local_trjectory.legend()

        plt.tight_layout()
        plt.ion()  # Enable interactive mode
        plt.show(block=False)

    def draw_frustum(self, ax, position, orientation):
        if self.scaling is None or self.scaling == 0:
            frustum_size = 0.1
        else:
            frustum_size = 0.1 / self.scaling

        x_dir = np.cos(orientation)
        z_dir = np.sin(orientation)
        o = np.array([x_dir, z_dir])
        tip = np.array(position)
        base_center = tip + o * frustum_size

        perp = np.array([-o[1], o[0]])  # rotate 90 deg in X-Z plane
        base_left = base_center + perp * (frustum_size / 2)
        base_right = base_center - perp * (frustum_size / 2)

        triangle = np.array([tip, base_left, base_right, tip])
        ax.plot(triangle[:, 0], triangle[:, 1], "g-")  # Green triangle for frustum

    def update(self, state, image_curr: np.ndarray, pose_history: list[np.ndarray]):
        # Update camera frame plot
        self.ax_img.cla()
        self.ax_img.imshow(image_curr, cmap="gray")
        self.ax_img.scatter(state.P[0, :], state.P[1, :], c="y", s=10)

        # Fix the axis limits so the image doesn't jump around
        img_height, img_width = image_curr.shape[:2]
        self.ax_img.set_xlim(0, img_width)
        self.ax_img.set_ylim(img_height, 0)
        self.ax_img.set_title(f"Frame No. {state.i} and Keypoints")
        self.ax_img.axis("off")

        # Update keypoint and candidate count plot (outlier plot)
        keypoint_count = state.P.shape[1]
        candidate_count = state.C.shape[1]

        self.time_steps.append(len(self.time_steps))
        self.keypoint_counts.append(keypoint_count)
        self.candidate_counts.append(candidate_count)

        self.ax_inliers.cla()
        self.ax_inliers.set_title("Keypoints and Candidate Count")
        self.ax_inliers.set_xlabel("Time (steps)")
        self.ax_inliers.set_ylabel("Count")

        self.ax_inliers.plot(
            self.time_steps, self.keypoint_counts, "g-", label="Keypoints"
        )
        self.ax_inliers.plot(
            self.time_steps, self.candidate_counts, "b-", label="Candidates"
        )
        self.ax_inliers.legend()

        # Update local trajectory plot
        self.ax_local_trjectory.cla()
        self.ax_local_trjectory.set_title("Local Trajectory (300 poses) and Landmarks")
        self.ax_local_trjectory.set_xlabel("X")
        self.ax_local_trjectory.set_ylabel("Z")

        cam_centers_local = []
        orientations_local = []
        for pose in pose_history[-300:]:
            R = pose[:3, :3]
            t = pose[:3, 3]
            c = -R.T @ t
            cam_centers_local.append([c[0], c[2]])
            orientations_local.append(get_camera_yaw_in_xz(pose))
        cam_centers_local = np.array(cam_centers_local)

        if len(cam_centers_local) > 0:
            # Plot local camera trajectory
            self.ax_local_trjectory.plot(
                cam_centers_local[:, 0],
                cam_centers_local[:, 1],
                "r-",
                label="Camera Trajectory",
            )

            self.draw_frustum(
                self.ax_local_trjectory,
                [cam_centers_local[-1][0], cam_centers_local[-1][1]],
                orientations_local[-1],
            )

            X_landmarks = state.X[[0, 2], :].T
            current_cam_pos = cam_centers_local[-1]  # Last camera pose
            distances = np.linalg.norm(X_landmarks - current_cam_pos, axis=1)
            self.scaling = 1.0 / np.mean(distances)

            # Filter and scatter landmarks
            valid_indices = distances <= self.range_threshold / self.scaling
            X_filtered = state.X[:, valid_indices]
            self.ax_local_trjectory.scatter(
                X_filtered[0, :],
                X_filtered[2, :],
                c="b",
                s=2,
                alpha=0.7,
                label="Landmarks",
            )

            # Dynamically set axes for the local trajectory
            last_cam = cam_centers_local[-1]
            radius_scaled = self.range_threshold / self.scaling
            self.ax_local_trjectory.set_xlim(
                [last_cam[0] - radius_scaled, last_cam[0] + radius_scaled]
            )
            self.ax_local_trjectory.set_ylim(
                [last_cam[1] - radius_scaled, last_cam[1] + radius_scaled]
            )

        self.ax_local_trjectory.set_aspect("equal", adjustable="datalim")
        self.ax_local_trjectory.legend()

        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)


def get_camera_yaw_in_xz(pose_3x4: np.ndarray) -> float:
    R = pose_3x4[:3, :3]
    z_world = R[2, :]
    x_component = z_world[0]
    z_component = z_world[2]
    return np.arctan2(z_component, x_component)
