# Lane Following Robot with ROS 2 & OpenCV

This project demonstrates a **lane-following robot simulation** using **ROS 2 (Humble)** and **OpenCV**.  
It processes camera (or test video) input in real time, detects lane lines, and publishes control commands to drive the robot using `geometry_msgs/Twist`.

---

## Tech Stack

- ROS 2 (Humble)
- Python 3
- OpenCV

---

## Project Structure

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
            
---

## Features

- Loads test video or camera feed using OpenCV
- Processes the frame using:
- Grayscale, Gaussian blur
- Canny edge detection
- Hough Transform for line detection
- Classifies left/right lane lines
- Draws dashed green lines for each lane
- Publishes Twist commands to /cmd_vel based on center error
- Built with ROS2 rclpy and a timer-based event loop

---

## How to Run

### 1. Build the workspace
cd ~/Lane_following_Project/lane_ws
colcon build
source install/setup.bash

### 2. Run the node
ros2 run lane_follower lane_follower_node\

---

## Project Result
(./lane_following_output.mp4)](./lane_following_output.mp4)

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
- GitHub: Gyuhyeok001
