# Project Log [21 Oct]

Source: [Project Log [21 Oct]](https://app.notion.com/p/400b182e8f0480fa8820c0023ac895ba)

---

# What did i do

1. The robot actual dimensions are
   Width = 17cm

   Length = 17cm

   Chasis Height = 18cm

   Wheel Separation = 12.5cm

   Changed the robot\_footprint and URDF, now the rviz visualization makes sense.
2. Tried to fix use\_rotate\_to\_heading but to no avail
   1. Put it back to false
3. yaw\_goal\_tolerance change to 3.14 aka no tolerance any heading works
   1. SUCCESSFUL STRAIGHT LINE GOAL
   2. Can be improved, it will start stop start stop,
4. FOCUSING ON TURNING TO A CORNER NOW!
5. Change the inflation radius.

# Improve cornering (now is the main goal before doing the 4 letter waypoints)

I was stuck at debugging the behavior server

- Gives a 0.0 ms-1 command at 35 hz
- Interferring with cmd\_vel from controller\_server
- During debugging i got this error though
  <p>
  In ROS 2 lifecycle management:

  - Transition 1 = Configure
  - unconfigured → inactive = Configure
  - inactive → active = Activate  
Your /planner\_server is already active, but the lifecycle manager is attempting to configure it again.

  That's why startup fails.

  - Fix
    My recommendation is to keep `autostart: False` for navigation because your project already has a custom navigation startup node.

We've already isolated the problem:

| Component | Observation |
| --- | --- |
| RPP | Stable 0.15 m/s |
| Velocity smoother | Stable 0.12 m/s |
| Behaviour server | Publishing zero commands at \~36 Hz |
| Recovery behaviours | Not visibly executing |
| Result | WaveRover repeatedly stops and starts |

An idle behaviour server should not continuously override navigation commands with zeros.

---

So basically, off all the behaviour and only keep one to see if it is giving the 0.0ms-1 command. 

IDEALLY, I should only have backup
</p>

> **Callout**
> 
My goal was to fix this cmd thing so its more smooth, also perhaps increase the translation speed to 0.25-0.30ms-1. Once more smooth i wanted to experiment the inflation radius and the cost-scaling factor.

[Image attachment: image.png — original media not included in this Markdown export]


# From now on

1. Fix command velocity, make it more smooth
2. Fix cornering
3. Give a 4 letter waypoint to follow
   1. Ensure its centered
4. Obstacle not found in the map, dynamic rerouting of plan.
