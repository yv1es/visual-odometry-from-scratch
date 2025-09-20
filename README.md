

<h1 align="center">📷 Visual Odometry from Scratch</h1>
<p align="center">
  Real-time monocular VO pipeline in Python + OpenCV  
</p>

<p align="center">
  <img src="https://github.com/yv1es/visual-odometry-from-scratch/raw/main/demo.gif" alt="Visual Odometry Demo" width="800"/>
</p>

---
This repository contains a monocular visual odometry (VO) pipeline built from scratch with OpenCV. It estimates camera poses and reconstructs 3D landmarks directly from video sequences. The system runs in real time on standard hardware and is structured into two main stages: Bootstrapping (initialization) and Continuous Operation (frame-by-frame tracking and mapping).

## Table of Contents

- [Working Principle](#working-principle)
  - [Bootstrapping](#bootstrapping)
  - [Continuous Operation](#continuous-operation)
- [Running the Pipeline](#running-the-pipeline)
- [Library Methods](#library-methods)
- [Demo](#demo)

---

## Working Principle

The VO pipeline operates in two main stages:

### Bootstrapping

- **Purpose:**  
  Initialize the pipeline by extracting an initial set of 3D landmarks.
  
- **Method:**  
  - **Keypoint Detection:**  
    Shi-Tomasi corner detection is applied to the first frame.
  - **Feature Matching:**  
    Detected corners are tracked using the Kanade-Lucas-Tomasi (KLT) method.
  - **Robust Estimation:**  
    The 8-point algorithm with RANSAC estimates the fundamental matrix. With camera calibration, the essential matrix is computed and decomposed into rotation and translation.
  - **Disambiguation:**  
    Among four possible solutions, the correct camera pose is selected by verifying the number of triangulated landmarks lying in front of both cameras.
  - **Output:**  
    An initial state is created, including keypoints, landmarks, the initial camera pose, and empty candidate sets.

### Continuous Operation

- **Purpose:**  
  Update the pipeline frame by frame to track keypoints and triangulate new landmarks.
  
- **Method:**  
  - **Localization:**  
    - **Tracking:**  
      KLT is used to track keypoints from the previous frame.
    - **Pose Estimation:**  
      The camera pose is computed using 2D–3D correspondences and P3P RANSAC.
  - **Landmark Triangulation:**  
    Candidate points are tracked and new landmarks are triangulated when the angle between camera rays exceeds a set threshold.  
    *Efficiency Tip:* Candidates from the same initial detection frame are grouped for joint triangulation.
  - **Candidate Selection:**  
    - **Detection:**  
      New candidate points are detected using Shi-Tomasi.
    - **Filtering:**  
      Only candidates that are a minimum distance away from existing keypoints are added to avoid duplication.
  - **Visualization:**  
    A 2D projection of the landmarks and trajectory is rendered in real time (this can be disabled for faster processing during deployment).

---

## Running the Pipeline

1. **Prepare the Dataset**  
   Download and extract the datasets into the `data` folder:
   - [PARKING](https://rpg.ifi.uzh.ch/docs/teaching/2024/parking.zip)
   - [KITTI](https://rpg.ifi.uzh.ch/docs/teaching/2024/kitti05.zip)
   - [MALAGA](https://rpg.ifi.uzh.ch/docs/teaching/2024/malaga-urban-dataset-extract-07.zip)

2. **Set Up the Environment:**
   - Install [Anaconda](https://www.anaconda.com/products/distribution).
   - Create the environment by running:
     ```bash
     conda env create -f environment.yml --prefix ./vo-env
     ```
   - Activate the environment:
     ```bash
     conda activate ./vo-env
     ```

3. **Run the Pipeline:**
   ```bash
   python main.py
   ```

4. **Select the Dataset:**  
   In `main.py`, set the `DATASET` variable to choose your dataset:
   ```python
   DATASET = Dataset.KITTI  # or Dataset.PARKING or Dataset.MALAGA
   ```

---

## Library Methods

The implementation leverages several key OpenCV functions:

- **Bootstrapping:**
  - `cv2.goodFeaturesToTrack`: Shi-Tomasi corner detection.
  - `cv2.calcOpticalFlowPyrLK`: Kanade-Lucas-Tomasi (KLT) tracking.
  - `cv2.findFundamentalMat`: 8-point (normalized) RANSAC for structure from motion (SfM).  
  - *(Alternative, not in use):* SIFT features and descriptors for correspondences.

- **Continuous Operation:**
  - `cv2.calcOpticalFlowPyrLK`: Tracking of keypoints and candidates.
  - `cv2.solvePnPRansac`: P3P RANSAC for 2D–3D camera localization.
  - `cv2.Rodrigues`: Conversion of rotation vectors to rotation matrices.
  - `cv2.triangulatePoints`: Landmark triangulation.
  - `cv2.goodFeaturesToTrack`: Detection of new candidate points.

---

## Demo

The pipeline was demonstrated on a Dell XPS-15 laptop with an Intel i7-12700H @ 4.7 GHz and 32GB RAM.  
Watch the demo on YouTube: [Visual Odometry Pipeline Demo](https://www.youtube.com/watch?v=Johjy9J9beY)
