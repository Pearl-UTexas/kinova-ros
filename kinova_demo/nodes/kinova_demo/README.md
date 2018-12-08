# Modifications from Original Repo
We made some changes in this fork in order to use these nodes for a horizontally mounted dual arm robot. 

## Files Changes
1. gravity_compensation_mode.py 
	- prefix changed to "right_arm_"/"left_arm_"
	- zero_torque_goal changed from all 180 to [90, 90, 180, 180, 180, 180, 180]
1. robot_control_modules.py.

## Using these nodes
Run these node with corresponding arguments and the gravity_compensation_mode shoudl set zero torque values for the arms.

