# Project Log [12 Nov]

---

# Actual Run results

> Missed out logs for the first run, but it went well. All center

| Run | Waypoint | Initial precision error | Final confirmed error | Time to arrival/stop* | Result |
| --- | --- | --- | --- | --- | --- |
| 2 | O | 6.2 cm | 1.4 cm | 15.9 s | Passed five AMCL checks |
| 2 | K | 6.1 cm | 1.3 cm | 37.0 s | Passed five AMCL checks |
| 2 | R | 5.7 cm | 1.3 cm | 26.0 s | Passed five AMCL checks |
| 2 | F | 4.8 cm | 1.9 cm | 16.1 s | Passed five AMCL checks |
| 3 | O | 10.7 cm | 0.7 cm | 27.5 s | Passed after additional correction |
| 3 | C | 7.0 cm | Not confirmed | 36.5 s | Settling timed out; last position estimate was 1.4 cm |
| 3 | A | — | — | — | Not attempted |
| 3 | G | — | — | — | Not attempted |

| Area | What we did | Outcome / limitation |
| --- | --- | --- |
| Bag recording | Created `record_debug.sh` with numbered, date-based folders | Saves under `/home/ubuntu/rb3302_final_proj_code/rosbags`, e.g. `01-12-11-26` |
| AMCL startup | Replaced the one-second service wait with a **15-second readiness retry** | Handles delayed discovery of `/request_nomotion_update` |
| Arrival confirmation | Increased confirmation from **two to five fresh AMCL estimates** | All must be within **2 cm of the target** and **1 cm of each other** |
| Stationary confirmation | Required **one second continuously stopped** before arrival checks | Added explicit settling checks |
| Settling diagnostics | Logged odometry age, linear speed, angular speed, and stationary duration | Identified fresh odometry with angular-rate excursions resetting the timer |
| Stopping threshold | Increased angular stopping threshold from **0.05 to 0.08 rad/s** | Temporary adjustment; a later run still encountered a settling timeout |
| Failed corrections | Added waypoint skipping after a confirmed precision-movement failure and confirmed stop | Records the missed letter and continues after three seconds |
| Reverse corrections | Added collision-checked **BackUp** steps for targets behind the robot | Steps capped at **3 cm**; RPP’s `allow_reversing: false` stays unchanged |
| Failure reporting | Added behaviour name, action status, and error code | Makes future movement failures easier to diagnose |
| Waypoint calibration | Produced updated YAML entries for **O, P, S, T, U, V, W, X** | Used your physically centred AMCL readings; O was revised twice |

## What went wrong during the actual run

**According to odometry, it didn’t remain stationary for one uninterrupted second within the four-second wait.**

**Run 3 failed at C because the robot did not pass the stationary check within four seconds.**

The sequence was:

1. O was reached and confirmed at **0.7 cm estimated error**.
2. At C, the first arrival-confirmation attempt failed when AMCL reported **2.3 cm**, outside the **2 cm tolerance**.
3. Further correction reduced the estimate to **1.4 cm**.
4. Before starting another five-update confirmation, the script required **one continuous second stationary**. That timer kept resetting and the four-second timeout expired.

**Odometry was fresh**, so this was not a missing-data problem. The resets indicate that at least one stopping condition failed between the printed log samples. The log does not establish whether that was actual slight movement or noisy velocity estimates.

The mission then stopped because **settling failures are still treated as stop conditions**; only confirmed movement failures are skipped. A and G were therefore not attempted.
