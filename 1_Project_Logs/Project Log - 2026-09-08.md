# What did I do
1. Setup VS code server inside the robot so that we can use code and not vim to edit files
2. Able to get the AMCL arrows 
	- Downloaded the nav2 pkg for particle cloud
	- The other one doesnt work because of message incompatibility
3. When  running the launch file, turn off the rviz thats trying to run on the robot, so you can only have it on the laptop
	`ros2 launch planning_rpp planning.launch.py rviz=false`
4. Learnt that changing the maps in args doesnt really work
	`ros2 launch planning_rpp planning.launch.py map="~/rb3302_final_proj_code/student_ws/map/pgpr_map.yaml` this supposed to work but i changed the launch file instead
