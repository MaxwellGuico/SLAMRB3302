# Project Log [4 Nov]

Source: [Project Log [4 Nov]](https://app.notion.com/p/40eb182e8f048012aac4c3246e265411)

---

# Whats wrong

| Priority | Problem | Evidence / Possible Cause | Possible Fix |
| --- | --- | --- | --- |
| 🔴 1 | **Controller drops from 20 Hz to \~1 Hz** | Happens shortly after navigation begins; possibly related to repeated replanning or blocking operations | Temporarily disable periodic replanning and compare controller performance |
| 🔴 2 | **Navigation goals fail** | `follow_path` times out or controller becomes unresponsive | Investigate custom behavior tree (`no_global_clear.xml`) and controller interaction |
| 🔴 3 | **EKF slows from 50 Hz to 17–20 Hz** | Happens during movement; occasional update delays exceeding 2 seconds | Investigate CPU scheduling and sensor processing; avoid changing EKF settings until root cause is identified |
| 🔴 4 | **TF extrapolation errors** | `map → odom` transform unavailable at requested timestamps | Check AMCL and EKF transform freshness; adjust tolerances only if justified |
| 🟠 5 | **Laser messages dropped** | Laser timestamps fall outside available TF history | Investigate TF timing, sensor timestamps, and processing delays |
| 🟠 6 | **Global costmap retains old obstacles** | `clearing: false` prevents raytracing from removing marked obstacles | Consider enabling clearing or implementing controlled obstacle persistence |
| 🟠 7 | **Potential processing bottleneck** | CPU cores around 68–70%; load average \~6.6 | Profile processes and callbacks; reduce unnecessary work if confirmed |
| 🟡 8 | **Repeated global replanning** | New paths passed to controller approximately every second | Test lower replanning frequency or disable periodic replanning temporarily |
