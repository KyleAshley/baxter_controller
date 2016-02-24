1) roslaunch baxter_controller baxter_controller.launch
2) roslaunch skeleton_markers skeleton_markers.launch
3) rosrun openni_tracker openni_tracker
4) rosrun baxter_controller gesture_tracker
5) rosrun baxter_controller Baxter_Controller.py

or

1) roslaunch baxter_controller DEMO_MIMIC.launch

Sit in front of camera with no-one else in view.
Run the above commands in order and control the arm movement/gripper.
