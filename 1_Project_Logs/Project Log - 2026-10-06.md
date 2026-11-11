# Project Log [6 Oct]

Source: [Project Log [6 Oct]](https://app.notion.com/p/3f1b182e8f048039ad8fc86d7a0196c4)

---

# Today’s progress

### What you've established so far

```txt
IMU mounted upside-down
        │
        ├── TF roll = 180° ✓
        │
        ▼
raw IMU:
CCW → negative ✓
        │
        ▼
EKF transforms it:
CCW → positive ✓
        │
        ▼
EKF integrates gyro ✓
        │
        ▼
BUT stationary gyro has
small residual bias
        │
        ▼
odom heading drifts
~5.6° / minute ✗
```
