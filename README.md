# allegro-lib
Simple Allegro lib interface/controller inherited from Seunsgu. 
###Dependencies
- [robot-toolkit](https://github.com/epfl-lasa/robot-toolkit) (you don't really need full robottoolkit, only the mathlib librayry)
- [motion-generators](https://github.com/epfl-lasa/motion-generators)

- peak-linux-driver-7.X

    Please visit http://peak-system.com


#### Controlling the hand:
To run the control interface:
```
$ rosrun allegrolib allegrolib
```

If you just want to simply open/close the hand, run theis node and follow the instructions on the command line:

```
$ rosrun allegrolib test_planner
```
