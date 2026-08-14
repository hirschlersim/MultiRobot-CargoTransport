# Startup order

Run these in order, each in its own terminal (with the catkin workspace sourced:
`source devel/setup.bash`).

`Image_processing_Yellow.py` and the `rpyc_*` scripts aren't registered as
catkin-installable nodes, so they can't be started with `rosrun` and are run
directly with `python3` instead. `rospack find` resolves each package's path
so these commands work from any catkin workspace, not just one user's machine.

1. Camera + AprilTag detection:
   ```
   roslaunch apriltag_ros continuous_detection.launch
   roslaunch video_stream_opencv webcamerausb.launch
   roslaunch apriltag_ros start_rviz.launch
   ```

2. DMPC controller (leader vehicle):
   ```
   rosrun leader_vehicle leader_vehicle
   ```

3. Object detection:
   ```
   python3 "$(rospack find detect_object)/scripts/Image_processing_Yellow.py"
   ```

4. Motor control (leader + follower):
   ```
   python3 "$(rospack find lego_control)/scripts/rpyc_leader.py"
   python3 "$(rospack find lego_control)/scripts/rpyc_follower.py"
   ```
