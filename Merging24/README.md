## How to run
```
python3 gv_socket_server.py
```
or to run the intersection algorithm:
```
python3 gv_socket_server.py intersection
```
### Gulliview
The version of gulliview this repository has used is in the gulliview2023-matei repository on the Gulliview pc.
Run the following command in the build folder:
```
sh startCamerasMerging.sh 2121
```
To use with the Advanced Mobility Model a port of all coordinates is needed and some small changes to how the messages are received with gullivutil.
Changes also needs to be made on the robots to handle the correct coordinates the same way. The algorithms themselves do not need to be changed.
### Bots merging
For each robot 4 terminals are needed, 1 that runs ros, 2 for handling UDP messages from Gulliview and this maneuver server, and 1 for the robots to follow a path and change speeds.
```
terminal 1:
rlaunch

terminal 2:
rosexp
mergelistenpos

terminal 3:
rosexp
mergelistenspeed

terminal 4:
rosexp
gomain / goramp
```
### Bots intersection
The paths for the robots when running the intersection is hardcoded to each robot instead of different pathnames as with main and ramp, this could probably be done better. To start their loop use the following instead of terminal 4 in merging:
```
rosexp
goint
```
The intersection algorithm needs more work to be more efficient, the calculation for changing speeds is not well implemented and the robots almost always need to stop since the path they are measuring to the middle of the intersection but the area for stopping is a lot bigger.
