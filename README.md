# Visual Odometry Pipeline

This project implements a **monocular visual odometry (VO) pipeline** from scratch using OpenCV building blocks. The pipeline estimates camera poses and reconstructs 3D landmarks from video sequences. It is designed to operate in two stages—**Bootstrapping** and **Continuous Operation**—and runs in real time on standard hardware. For a detailed discussion of the pipeline’s inner workings, please refer to the _Visual Odometry Pipeline Report_ section below.

---

## Table of Contents

- [Working Principle](#working-principle)
  - [Bootstrapping](#bootstrapping)
  - [Continuous Operation](#continuous-operation)
- [Running the Pipeline](#running-the-pipeline)
- [Library Methods](#library-methods)
- [Screencasts](#screencasts)

---

## Working Principle

The VO pipeline operates in two main stages:

### Bootstrapping

- **Purpose:** Initialize the pipeline by extracting an initial set of 3D landmarks.
- **Method:**
  - **Keypoint Detection:** Initially, Shi-Tomasi corner detection is used.
  - **Feature Matching:** Shi-Tomasi corners are tracked using the Kanade-Lucas-Tomasi (KLT) method.
  - **Robust Estimation:** The 8-point algorithm with RANSAC is used to estimate the fundamental matrix. Camera calibration then yields the essential matrix, which is decomposed into rotation and translation.
  - **Disambiguation:** Among four possible solutions, the correct camera pose is selected by checking how many triangulated landmarks lie in front of both cameras.
  - **Output:** An initial state consisting of keypoints, landmarks, camera pose, and empty candidate sets is created.

### Continuous Operation

- **Purpose:** Update the pipeline frame by frame to track keypoints and triangulate new landmarks.
- **Method:**
  - **Localization:** 
    - **Tracking:** Use KLT to track keypoints from the previous frame.
    - **Pose Estimation:** Compute the camera pose using 2D–3D correspondences and P3P RANSAC.
  - **Landmark Triangulation:**
    - Track candidate points and triangulate new landmarks when the angle between camera rays exceeds a threshold.
    - **Efficiency:** Grouping candidates with the same initial detection frame allows joint triangulation.
  - **Candidate Selection:**
    - **Detection:** New candidates are detected using Shi-Tomasi corner detection.
    - **Filtering:** Candidates are added only if they are a minimum distance away from existing keypoints to avoid duplication.
  - **Visualization:** For development and evaluation, a 2D projection of the landmarks is rendered to provide real-time feedback, though it can be disabled for higher processing speeds.

---

## Running the Pipeline

1. **Prepare the Dataset**  
   Download the datasets and extract them into the `data` folder:
   - [PARKING](https://rpg.ifi.uzh.ch/docs/teaching/2024/parking.zip)
   - [KITTI](https://rpg.ifi.uzh.ch/docs/teaching/2024/kitti05.zip)
   - [MALAGA](https://rpg.ifi.uzh.ch/docs/teaching/2024/malaga-urban-dataset-extract-07.zip)

2. **Set up the Environment:**
   - Install [Anaconda](https://www.anaconda.com/products/distribution).
   - Create the environment:
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
   DATASET = Dataset.KITTI # or Dataset.PARKING or Dataset.MALAGA
   ```

---

## Library Methods

The implementation leverages several key OpenCV methods:

- **Bootstrapping:**
  - `cv2.goodFeaturesToTrack`: Shi-Tomasi corner detection
  - `cv2.calcOpticalFlowPyrLK`: Kanade-Lucas-Tomasi tracking
  - `cv2.findFundamentalMat`: 8-point (normalized) RANSAC for structure from motion (SfM)
  - *(Alternative, no longer used):* SIFT features and descriptors for correspondences

- **Continuous Operation:**
  - `cv2.calcOpticalFlowPyrLK`: Keypoint and candidate tracking
  - `cv2.solvePnPRansac`: P3P RANSAC for 2D–3D camera localization
  - `cv2.Rodrigues`: Rotation vector to matrix conversion
  - `cv2.triangulatePoints`: Landmark triangulation
  - `cv2.goodFeaturesToTrack`: Shi-Tomasi for detection of new candidate points

---

## Demo

The recording was made on a Dell XPS-15 laptop with an Intel i7-12700H @ 4.7 GHz and 32GB RAM. 
Link to youtube https://www.youtube.com/watch?v=Johjy9J9beY
