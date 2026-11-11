# Project Log [26 Oct]

Source: [Project Log [26 Oct]](https://app.notion.com/p/404b182e8f0480f09d96cbe5114ccacc)

---

# Test the Replanner first tho…

# Then Add this

[File attachment: letter_waypoint.py — original media not included in this Markdown export]

In `CMakeLists.txt`:

```txt
install(
  PROGRAMS scripts/letter_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME letter_waypoints
)
```

In `package.xml`, place these inside the `<package>...</package>` section:

```txt
<exec_depend>rclpy</exec_depend>
<exec_depend>action_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav2_msgs</exec_depend>
```

Also ensure the script is executable:

```txt
chmod +x scripts/letter_waypoints.py
```

Then rebuild and source:

```txt
cd ~/rb3302_final_proj_code/student_ws
source /opt/ros/jazzy/setup.bash
source ~/rb3302_final_proj_code/robot_ws/install/setup.bash

colcon build --symlink-install --packages-select planning_rpp
source install/setup.bash
```

Verify installation:

```txt
ros2 pkg executables planning_rpp
```

You should see:

```txt
planning_rpp letter_waypoints
```

Then run it:

```txt
ros2 run planning_rpp letter_waypoints A F K X
```

[Image attachment: image.png — original media not included in this Markdown export]

[Image attachment: image.png — original media not included in this Markdown export]



# What i worked on

1. I added letter\_waypoints.py
   1. worked on tolerance of 0.15
      1. Off by 4cm in x, 20cm in y
   2. **I changed goal tolerance to 0.05 without testing**
2. Replanning behaviour works
   1. Only if the obstacle is big enough, if there are “holes” through the obstacle, it will try to plan a path through it, then it does the recovery behaviour
3. Re enabled recovery behaviour
   1. Again 1.57 rad spin actually isnt a 90 degree spin, turning for the robot still needs some work.
   2. Added a custom xml behaviour tree, removing the clearing of global cost
      > **Callout**
> 
      Everytime it does the replan, it will “forget” where the obstacles are, causing it go back to the same path, re encountering the same problem.

      I tried putting local costmap clearing to false, it didnt work well, it continually went into recovery mode
      

# What needs to work

In order of priority

1. Replanning
2. Improved angular velocity
   1. Inclusive of recovery behaviour turning spin behavior
