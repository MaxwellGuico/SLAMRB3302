# Project Log [11 Nov]

Source: [Project Log [11 Nov]](https://app.notion.com/p/415b182e8f0480d8b375ff85c8de4be6)

---

| Problem | What we changed | Result / finding |
| --- | --- | --- |
| **EKF and controller slowing down** | Reduced costmap overhead and restored obstacle clearing. | EKF returned to roughly **50 Hz**; uninterrupted navigation could achieve around **20 Hz**. |
| **Low command-topic frequency** | Compared command gaps with navigation and recovery events. | Low averages included collision stops, recoveries and pauses—not just slow controller computation. |
| **Repeated collision/recovery loops** | Updated recovery XML to force replanning instead of repeatedly accepting the same unusable path. | Addressed one source of repeated stopping. |
| **Cutting corners near obstacles** | Changed inflation cost scaling to **3**, with the matching RPP setting. | You reported better obstacle clearance. |
| **Robot physically turning too slowly** | Tested commanded versus measured angular velocity; added driver **angular gain 3**, retaining an output cap. | Physical turning response improved substantially. |
| **Poor final approach and low-speed slip** | Added a precision mode: Nav2 approaches to about **10 cm**, then hands over to collision-checked `Spin` and short `DriveOnHeading` actions. | More deliberate alignment and approach, with AMCL checking estimated position between movements. |
| **Precision alignment oscillation** | Reduced angular correction size, adjusted heading tolerance and added reversal/no-progress guards. | Reduced repeated left/right corrections and bounded unsuccessful attempts. |
| **Mission stopping on stale AMCL data** | Added startup waiting for both pose and scan, clearer error messages, refreshes after pauses and periodic refresh requests during navigation. | Addressed freshness-check failures; retained checks for genuinely unavailable data. |
| **Precision failure ending the whole mission** | Added the original waypoint script’s flow: confirm stopping, announce the missed goal, wait three seconds and attempt the next waypoint. | Recoverable precision failures can be skipped; missed goals are reported at the end. |
| **Waypoint coordinate mismatch** | Sampled all **24 physical cell centres** and put their coordinates into shared YAML used by both waypoint scripts. | Calibration alone gave only slight improvement, indicating another underlying error. |
| **Remaining 5–10 cm physical positioning error** | Corrected `lidar_from_imu_x` from **`−0.052`**** to ****`+0.052`**** m**. | **The decisive fix.** The sign error represented a **10.4 cm discrepancy** in the modelled sensor position. You retested and reported substantially better accuracy. |
| **Alternative grid-only navigation** | Explored straight cell-centre routes with stops at turns. | Set aside; continued with the precision waypoint approach. |

**The biggest improvements came from correcting physical turning response and fixing the LiDAR transform.** The final positioning improvement is confirmed by your test; a numerical final accuracy still needs measured stopping errors.
