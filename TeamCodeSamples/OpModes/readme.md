# Op Modes

The op modes in this folder are examples of autonomous and teleop strategies. They are intended to show you various code structures and design solutions from past and current games.

Even though the game changes every season there are some basic functions that your robot will need to perform. Some examples:

Task | Description | Hardware/Software
--- | --- | ---
Navigation | In autonomous the robot needs to navigate around the field on its own. | camera, imu, odometry, distance sensor, color sensor
Object Detection | A common game theme has been the requirement to find a game piece somewhere on the field. | camera, color sensor, distance sensor, IR sensor
Drive Train | The game may emphasize speed, agility, climbing or some other attribute. But you will still need a drive train. | motors, encoders, gears
Arm | There is usually a requirement to lift or reach, which is some kind of arm. | motors, gears, servos, limit switch, touch sensor
Grabber | Picking up on or more game pieces, and sometime launching them is a fairly common theme of games in the past. | servos, motors, touch sensor, color sensor
