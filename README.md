# autocar
Final year project - Autonomous car with signal and pedestrian detection
# Setup
```
git clone https://github.com/pranavmp-10-000/autocar/
cd autocar
git lfs pull
cd catkin_ws/src
git clone https://github.com/OTL/cv_camera
cd ../
catkin_make
```
# Run
## Roscore
In a separate terminal, run ```roscore```
## Node run
Run the below nodes simultaneously 
- To run the camera node
  ``` rosrun cv_camera cv_camera_node ```
- To run Traffic sign detection node
  ``` rosrun dl_detection traffic_light_detection.py __name=traffic_signal_detection ```
