# allegro-lib
Simple Allegro lib interface/controller inherited from Seunsgu. Depends on Mathlib and Motion_generators from RobotToolkit.

###Dependencies
- [robot-toolkit](https://github.com/epfl-lasa/robot-toolkit) (from epfl-lasa)
- [motion-generators](https://github.com/epfl-lasa/motion-generators) (from epfl-lasa)

#### Controlling the hand:
To run the control interface:
```
$ rosrun allegrolib allegrolib
```

If you just want to simply open/close to grab a tool or whatever,  run theis node and follow the instructions on the command line:

```
$ rosrun allegrolib test_planner
```
