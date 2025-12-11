# Lane Following Robot with ROS 2 & OpenCV

This package demonstrates a simple lane detection pipeline using **ROS 2**, **OpenCV**, and **Python**.  
It reads a sample road video from the package, detects lane markings, and records an annotated output video.

---

## Tech Stack

- ROS 2 (Foxy / Humble)
- rclpy
- OpenCV
- Python 3.8+
- NumPy
  
---

## Project Structure
```test
lane_ws/
└── src/
    └── lane_follower/
        ├── package.xml
        ├── setup.py
        ├── resource/
        │   └── lane_follower
        ├── test/
        │   └── lanevideo.mp4
        └── lane_follower/
            ├── __init__.py
            └── lane_follower_node.py
```            
---

## Features

- Reads a test lane video (`lanevideo.mp4`) from the package share directory
- Applies grayscale, Gaussian blur, Canny edge detection, and Hough transform
- Draws detected lane segments as dashed lines on each frame
- Displays the processed video in a window
- Saves the annotated video as `lane_following_output.mp4`

---

## How to Run

### 1. Build the workspace
cd ~/lane_following_Project/lane_ws        # adjust to your actual workspace path
colcon build
source install/setup.bash

### 2. Run the node
ros2 run lane_follower lane_follower_node

---

## Project Result
[![Lane Tracking Output](./lane_following_output.mp4)](./lane_following_output.mp4)

---

## Future Improvements

- Integrate with real robot hardware
- Simulate using Gazebo or RViz
- Replace video input with real-time USB camera
- Tune Twist output using PID control
- Add launch files and parameter support


---

## License
This project is licensed under the MIT License.

---

## Author
- GitHub: [Gyuhyeok001](https://github.com/Gyuhyeok001)
