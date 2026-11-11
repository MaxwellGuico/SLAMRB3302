# Project Log [20 Oct]

Source: [Project Log [20 Oct]](https://app.notion.com/p/3ffb182e8f048046a343f8421dfc0218)

---

# What did WE do

<ol>
<li>
Change goal tolerance 
</li>
<li>
I realized that there are two IMU

1. One waverover
2. One yesense
   1. Gives quaternions to the EKF
   2. By putting external topics to true
      1. We can get /euler\_only angles.

   > **Callout**
> 
   The yesense yaw is so much more accurate than the Waverover IMU

   Thinking if we can use the yesense for more like for linear

   Currently we are fusing the angular z and the yaw absolute heading

   
   
</li>
<li>
Changed the amcl parameters, and once it moves for a prolonged period of time, it is able to localise so much better
</li>
<li>
use\_rotate\_to\_heading set to true

1. The commanded rotation velocity is 0.04, deadband is 0.05

## What is `use_rotate_to_heading` in Nav2?

`use_rotate_to_heading` is a parameter in the **Regulated Pure Pursuit (RPP) controller**.

It determines whether the robot should **rotate in place to face the path before moving forward**, when its heading differs significantly from the direction of the path.
</li>
</ol>

```txt
true
```

Robot may stop and rotate before following the new path direction

```txt
false
```

Robot generally follows a curved trajectory towards the path



# What to do next

<ol>
<li>
I think need to improve the deadband for vx and and angular velocity
</li>
<li>
There has to be way to visualise what is the path that the robot wants to talk 
</li>
<li>
Change the use\_rotate\_to\_heading

```txt
use_rotate_to_heading: true
rotate_to_heading_min_angle: 0.785
rotate_to_heading_angular_vel: 0.3
```
</li>
</ol>
