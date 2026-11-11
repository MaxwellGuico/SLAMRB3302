# Project Log [5 Nov]

Source: [Project Log [5 Nov]](https://app.notion.com/p/40fb182e8f0480539843cbe194ad44cf)

---

# What did i do

1. Noticed that while other robots are there, hard to test
2. Obstacles given by TAs covers full one square
3. Fixed replanning
4. Experimented with spin behaviour
   1. To off it, just comment in the custon .xml file and it works
      Exposed a problem with shim controller during replanning, it doesnt turn (look at What to do next)

# What to do next

1. Find someway to improve the turning velocity during runs because it affect linear velocity
   1. Right now its going at a good speed translationally, but it tends to “crash” onto a wall  
I think its not turning fast enough
2. create ./kill.sh that kills ros2 nodes in case of crashes
3. I think the spin recovery is good but sometimes it spins the wrong way.
4. Improve letter\_waypoint, when we kill the node, it should gracefully kill goals, right now if i kill the node, it will crash nav2 stack
   [File attachment: letter_waypoint.py — original media not included in this Markdown export]
5. Havent check goal tolerance
   1. Just go to each letter
6. Apparently the fact that spin behavior accidentally helped the shim controller is interesting

[Image attachment: image.png — original media not included in this Markdown export]

```javascript
 rotate_to_heading_once: false
```

7. add a shell command to open rviz with config file automatically called nav\_rviz.sh

# Stuff i havent thought about

1. Skip failed landmarks according to the grading procedure**Potential mismatch**`stop_on_failure: false` allows automatic continuation. The brief requires the team to **verbally declare a skip before reaching the next landmark**.

**My priorities before calling it complete would be:**

1. Run three full mock graded missions: four destinations, three-second stops, no interaction, under 180 seconds. Record contacts and centre-label coverage.
2. Test the difficult cases: an unexpected obstacle, repeated destinations, and different starting bearings within ±45°.
3. Make failed-waypoint handling clear: automatic continuation must not silently bypass the required verbal skip declaration.
4. Abit confused with the goal failed behavior
5. Amount of work done section: Lets add the project logs into the github, and add it in the report saying. All the logs should be converted into md files. Dont forget to include the obsidian ones
6. the starting of imu sometimes not very reliable, we should probably fix that FIRST.

# new letter\_waypoint.py

changes:

1. Improve letter\_waypoint, when we kill the node, it should gracefully kill goals, right now if i kill the node, it will crash nav2 stack
2. Add countdown when robot moves (3 seconds)
3. Add skipping if goal failed
   [File attachment: letter_waypoint.py — original media not included in this Markdown export]

# Warning: Revert back changes on the robot

[Image attachment: image.png — original media not included in this Markdown export]



# WARNING: i put no spin behavior in the report.
