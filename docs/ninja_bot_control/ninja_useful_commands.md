# Useful Git Commands

`git status`  
Shows current branch and changed files.

`git branch`  
Shows local branches. The current branch has `*`.

`git branch -a`  
Shows local branches and GitHub branches.

`git fetch --all --prune`  
Updates the branch list from GitHub.

`git switch main`  
Switch to `main`.

`git switch ninja`  
Switch to `ninja`.

`git switch -c ninja`  
Create a new local `ninja` branch. Only use if it does not exist yet.

`git switch -c ninja origin/ninja`  
Get the `ninja` branch from GitHub on a new computer. Only use once per computer.

`git pull`  
Download newest changes for the branch you are currently on.

`git add .`  
Stage all changed files.

`git add src/package_name`  
Add one specific ROS2 package folder.

`git add src/ninja_bot_control/ninja_bot_control/esp32_bridge.py`  
Add one specific file inside the ninja package.

`git commit -m "message"`  
Save staged changes locally.

`git push`  
Upload committed changes to GitHub.

`git push -u origin ninja`  
First push when you created a new branch locally and it does not exist on GitHub yet. Usually only once.

Note: Git stays on the last branch you switched to until you switch again.

`sudo apt install gh`
` gh auth login`

Authenticate for saving in git

`git fetch  --all`
`git switch Thanish` - to the branch you want to merge with
`git merge origin/Eimund` - so the branch you merge from will overwrite the current branch so Thanish 


## ROS2 Ninja Setup

`cd ~/Eurobot2026MTP`  
Go to workspace.

`source /opt/ros/jazzy/setup.bash`  
Load ROS2 Jazzy.

`colcon build --packages-select ninja_bot_control`  
Build ninja package.

`source install/setup.bash`  
Load workspace after building.

`ros2 pkg executables ninja_bot_control`  
Check available ninja nodes.


## ESP32 Direct Serial Test

`ls /dev/ttyUSB* /dev/ttyACM*`  
Find ESP32 port.

`python3 -m serial.tools.miniterm /dev/ttyUSB0 115200`  
Open direct serial to ESP32.

`Ctrl + ]`  
Exit miniterm.

`Ctrl + AltGr + 9`  
Exit miniterm on Nordic keyboard.

Useful ESP32 commands:

```text
ping
settings
stop
watchdog off
m1 60
m1 0
m2 60
m2 0
watchdog on
timeout 300
```


## ROS2 Ninja Bridge Test

Terminal 1:

`ros2 run ninja_bot_control esp32_bridge --ros-args -p port:=/dev/ttyUSB0`  
Run ESP32 bridge.

Terminal 2:

`ros2 topic echo /ninja/telemetry`  
Read bridge/ESP32 messages.

Terminal 3:

`ros2 topic pub --once /ninja/esp32_cmd std_msgs/msg/String "{data: 'ping'}"`  
Test connection.

`ros2 topic pub --once /ninja/esp32_cmd std_msgs/msg/String "{data: 'settings'}"`  
Show ESP32 settings.

`ros2 topic pub --once /ninja/esp32_cmd std_msgs/msg/String "{data: 'stop'}"`  
Stop motors.

`ros2 topic pub --once /ninja/esp32_cmd std_msgs/msg/String "{data: 'motors 80 80'}"`  
Drive both motors slowly.


## Ninja Command Terminal

Terminal 1:

`ros2 run ninja_bot_control esp32_bridge --ros-args -p port:=/dev/ttyUSB0`  
Bridge must run first because it owns USB serial.

Terminal 2:

`ros2 run ninja_bot_control ninja_cmd_terminal`  
Open short command terminal.

Then type commands directly:

```text
ping
settings
stop
m1 60
m1 0
motors 80 80
```

## Ninja commands 

```Esp32

ping
help
settings
resetsettings
```

```Motor tuning

trim <left_trim> <right_trim>
minpwm <0-255>
maxpwm <0-255>
ramp <0-255>
watchdog on
watchdog off
timeout <50-5000>
testmotor <motor> <start> <end> <step>

Eksempel:

trim 0 -5
minpwm 50
maxpwm 120
ramp 10
timeout 300
testmotor 1 30 120 10
testmotor 2 30 120 10
```

```Motors

forward
backward
left
right
reverseleft
reverseright
stop
m1 <-255 to 255>
m2 <-255 to 255>
motors <left> <right>

Eksempel: 
m1 60
m1 0
m2 60
m2 0
motors 80 80
motors -80 -80
motors -80 80
motors 80 -80
stop
```

```Gripper / tilt

startposition
neutralposition
eating
stopeating
twocrates
onecrate
tiltup
tiltdown
release
tilt <0-180>
grip <0-180>
```

```VLX sensors

vlx         - vlx toggles continuous VLX streaming on/off.
vlxstatus
```

```Voltage

voltage
voltagestream   - voltagestream toggles continuous voltage streaming on/off.
```



## ROS2 cmd_vel Test

Forward:

`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --rate 10`

Turn:

`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" --rate 10`

Stop:

`ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"`


## Useful ROS2 Checks

`ros2 node list`  
List running nodes.

`ros2 topic list`  
List topics.

`ros2 node info /esp32_bridge`  
Check bridge publishers/subscribers.

`ros2 topic info /ninja/esp32_cmd`  
Check command topic.



## Code Dependency Reminder

Arduino owns hardware: motors, pins, servos, sensors.  
`esp32_bridge.py` owns USB serial and converts `/cmd_vel` to `motors left right`.  
`ninja_cmd_terminal.py` only forwards typed commands to `/ninja/esp32_cmd`.

Change only Arduino internals = upload Arduino only.  
Change Arduino command names like `motors` or `stop` = update Arduino and `esp32_bridge.py`.  
Change Python files = run `colcon build --packages-select ninja_bot_control` and `source install/setup.bash`.


## Terminal Shortcuts

`Ctrl + C` stop running program.  
`Ctrl + L` clear terminal.  
`Ctrl + A` start of line.  
`Ctrl + E` end of line.  
`Ctrl + U` delete before cursor.  
`Ctrl + K` delete after cursor.  
`Ctrl + W` delete previous word.  
`Ctrl + R` search command history.  
`Ctrl + Shift + T` new terminal tab.  
`Ctrl + Shift + C` copy.  
`Ctrl + Shift + V` paste.  
`Super + Left/Right` snap window left/right.