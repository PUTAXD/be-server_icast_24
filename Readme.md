## Back-End Basestation 2023

ROS server basestation IRIS 2023

## Dependencies

- rosbridge

## How To Compile

```
catkin_make
```

if error is not about dependencies requirement, just keep running command `catkin_make` till succed.

## How To Run

```
roslaunch basestation basestation.launch
```

## Common Mistake

Make sure port UDP in config and port 9090 is freely to use (not binding to any process)