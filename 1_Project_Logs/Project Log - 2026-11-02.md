# Project Log [2 Nov]

Source: [Project Log [2 Nov]](https://app.notion.com/p/40cb182e8f04809898d1e8e846ed5f9a)

---

# What did I do

- [ ] Increased Angular Vel = 1.5, Angular Accel = 12 and Linear Vel = 0.25
- [ ] Once it sees another obstacle, it gets lost
  - [ ] I think recovery needs to improve
- [ ] LEARNT that if i have integer for param that requires double, it crashes everything.
- [ ] I tried increasing linear speed to 0.4 (max verstappen)
  - [ ] Did not work as the turning is slower than it moving linearly
  - [ ] Tends to ‘crash’
- [ ] put FollowPath.closed\_loop to true
  - [ ] I think it works, it increased to 1.5rads-1
- [ ] Inflation radius to 0.4 and cost factor to 3.0







## Things to change

<ol>
<li></li>
</ol>

File: \~/rb3302\_final\_proj\_code/student\_ws/src/planning\_rpp/config/nav2\_params.yaml

┌──────┬──────────────────────────┬──────────────────────────┐

│ Line │       Change from        │        Change to         │

├──────┼──────────────────────────┼──────────────────────────┤

│ 92   │ transform\_tolerance: 0.1 │ transform\_tolerance: 0.3 │

├──────┼──────────────────────────┼──────────────────────────┤

│ 118  │ transform\_tolerance: 0.3 │ transform\_tolerance: 0.5 │

├──────┼──────────────────────────┼──────────────────────────┤

│ 169  │ transform\_tolerance: 0.3 │ transform\_tolerance: 0.5

2. Recovery change the behaviour
   <ol>
   <li>
   Remove the spin, just go back a smaller distance
   </li>
   <li></li>
   </ol>

Step 2: lighten the EKF

File: \~/rb3302\_final\_proj\_code/student\_ws/src/planning\_rpp/launch/

[planning.launch.py](https://planning.launch.py/)

┌──────┬────────────────────────────┬─────────────────────────────┐

│ Line │        Change from         │          Change to          │

├──────┼────────────────────────────┼─────────────────────────────┤

│ 92   │ 'frequency': 50.0,         │ 'frequency': 20.0,          │

├──────┼────────────────────────────┼─────────────────────────────┤

│ 96   │ 'print\_diagnostics': True, │ 'print\_diagnostics': False, │

├──────┼────────────────────────────┼─────────────────────────────┤

│ 134  │ 'imu0\_queue\_size': 50,     │ 'imu0\_queue\_size': 5,       │

└──────┴────────────────────────────┴─────────────────────────────┘

- Not sure about reducing the frequency
- im0 queue size defo sounds like we can decrease
