# Project Log [22 Oct]

Source: [Project Log [22 Oct]](https://app.notion.com/p/401b182e8f048052ab00ea8d173f1742)

---

# What did I do

1. Re-enabled the behaviour tree stack → It was causing error when i only have spin
   1. Its not connected to /cmd\_vel yet → still in debugging mod (imo, works better without it for now)
2. Added rotation shim controller
   1. Rotate to path before translating



# Current Problem

### Fix angular velocity issue

> **Callout**
> 
<ol>
<li>
Its rotating quite slowly. Ideally should rotate at 0.5. After fixing this problem, i want to rotate at 1.50 rads-1. 
</li>
<li>
As it rotates, its localisation becomes inaccurate

1. Maybe we should do decrease min\_a under AMCL
</li>
</ol>


## What was discovered

| Parameter / Measurement | Value |
| --- | --- |
| `rotate_to_heading_angular_vel` | 0.5 rad/s |
| `max_angular_accel` | 4.0 rad/s² |
| Rotation Shim output | 0.2–0.3 rad/s |
| Velocity smoother output | 0.2–0.3 rad/s |
| EKF angular velocity | 0.2–0.3 rad/s |
| Global replanning | 1 Hz |

### 2. Why we're using `rqt_plot`

We want to compare two signals simultaneously:

**Commanded angular velocity:**

```txt
/cmd_vel_nav_raw/angular/z
```

**Estimated angular velocity:**

```txt
/odom/twist/twist/angular/z
```

We're trying to determine whether the fluctuations originate in the Rotation Shim or its odometry feedback.

> **Callout**
> 
No clue why both lines wont show in the same graph




#### What to look for

| Observation | Interpretation |
| --- | --- |
| Both curves fluctuate together | The robot's estimated rotation follows the changing commands |
| Command fluctuates, odometry remains smooth | Controller output is changing independently of measured motion |
| Odometry fluctuates before the command changes | Odometry feedback may be influencing the controller |
| Command repeatedly resets around 1 Hz | Global replanning may be interrupting the rotation |

[Image attachment: image.png — original media not included in this Markdown export]





Also sometimes IMU not online, just restart waverover driver
