# Op Modes

The op modes in this folder are examples of autonomous and teleop
strategies. They are intended to show you various code structures and
design solutions from past and current games.

An understanding of op modes is a requirement for creating a successful
robot control app. The first thing to understand is the difference
between iterative and linear op modes. Linear is easier to understand
but iterative can give you more control. Either style will work. The
choice is made by you based on your robot and its coding requirements.

The following op modes in the FTC sdk, in this folder,
[external.samples](https://github.com/FIRST-Tech-Challenge/FtcRobotController/tree/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples) illustrate the 2 kinds of op modes.

1. BasicOpMode_Linear - This is a simple 2 motor linear teleop op mode.
   Blocks creates linear op modes by default. Look at the next op mode
   to see an example of iterative op modes.
2. BasicOpMode_Iterative - The sdk platform loops automatically for you.
   This is an example of an *event driven* programming model.

### Holonomic Drive Train

Since mecanum wheels and holonomic drive trains have become common,
[EdinaFTCOmniTest.java](EdinaFTCOmniTest.java) in ths folder, is a great
place to begin your understanding and testing of holonomic drives. This op mode
should be a permanent part of your test toolkit for verifying your
holonomic drive train setup.

### Test opModes

In order to verify and test hardware you should have a collection of test opModes. This can be very useful when you run into those inevitable "it used to work" situations. Feel free to copy these and modify them to your team's specific needs.

Even though the game changes every season there are some basic functions
that your robot will almost always need to perform.

Some examples:

| Task             | Description                                                                                                     | Hardware/Software                                    |
|:-----------------|:----------------------------------------------------------------------------------------------------------------|:-----------------------------------------------------|
| Navigation       | In autonomous the robot needs to navigate around the field on its own.                                          | camera, imu, odometry, distance sensor, color sensor |
| Object Detection | A common game theme has been the requirement to find a game piece somewhere on the field.                       | camera, color sensor, distance sensor, IR sensor     |
| Drive Train      | The game may emphasize speed, agility, climbing or some other attribute. But you will still need a drive train. | motors, encoders, gears                              |
| Arm              | There is usually a requirement to lift or reach, which is some kind of arm.                                     | motors, gears, servos, limit switch, touch sensor    |
| Grabber          | Picking up one or more game pieces, and sometime launching them is a fairly common theme of games in the past.   | servos, motors, touch sensor, color sensor           |

<details><summary>Autonomous Configuration</summary>

## Autonomous Configuration

This class is designed to provide a way for teams to configure
autonomous op modes to perform tasks differently based on information
that is learned just before starting a match in competition.

### Some examples

- Change your navigation path based on your alliance color.
- Select the starting location of you robot.
- Select which tasks your robot will perform in the autonomous session.
- Select the parking location at the end of autonomous.
- Define any delay before starting autonomous.

This *AutonomousConfiguration* class can be modified to meet the
requirements of your team and robot, and of course for next season's
game.

The "quick-and-dirty" way to manage these options is to create a bunch
of different autonomous op modes using "creative" names to identify
their capabilities. With the 6 configuration options in this sample
class you would need at least **64** different op modes to cover all possible options.

**2 big problems with that strategy**:

1. Can the driver reliably pick the correct opmode from the list on the
   phone in a dark gym with hundreds of cheering fans and extremely loud
   music blasting from the PA system?
2. When you make a change in your code can you be sure not to make a
   mistake when adding the change to all of the autonomous opmodes?

### Code

- AutonomousConfiguration.java - This is the configuration class. Add
  this to your team code folder and change it to meet your requirements. **Note:** This class uses a gamepad extension from [this repository](https://github.com/HF-Robotics/TntFtcCore). The classes used are in [this folder](https://github.com/HF-Robotics/TntFtcCore/tree/master/corelib/src/main/java/com/ftc9929/corelib/control). This code handles the issue caused by looping when you are detecting a single key press. You could also use your own code ot another library like FtcLib.
- AutonomousOptions.java - This class holds all of your options for the current game. It would change for new games each year.
- ReadWriteAutoOptions.java - This class reads and writes the AutonomousOptions object to a file on the control hub. It can then be used by other op modes, including teleop.
- RHSConfigMenu.java - This is a sample op mode that uses AutonomousConfiguration.
- RHSConfigMenuLinear.java - This is a linear op mode that uses AutonomouseConfiguration.

</details>

<details><summary>State Machine</summary>

## State Machine

*State machine*, *finite state machine*, *finite automation* all end up
at *state machine* in
[WikiPedia](https://en.wikipedia.org/wiki/Finite-state_machine). Take a
look for a formal description of state machine. And of course
[gmZero](https://gm0.org/en/latest/docs/software/finite-state-machines.html)
has a section on state machines.

In FTC robot code, a state machines can help to organize and make it
easier to enhance and update your op modes.

### Code

***Note: The code here is due for an update. (September, 6, 2023)***

- RHSAutoStateMachineGyro.java - This is an autonomous op mode that uses a
state machine as well as some other useful coding strategies.

#### Autonomous

This state diagram goes with the op mode.

```mermaid
stateDiagram-v2
  [*] --> Initial
  state if_state <<choice>>
  Initial --> if_state
  if_state --> Park_In_Warehouse
  if_state --> Park_In_Storage
  Park_In_Warehouse --> Deliver_Duck
  Park_In_Storage --> Deliver_Duck
  Deliver_Duck --> Deliver_Freight
  state if_state2 <<choice>>
  Deliver_Freight --> if_state2
  if_state2 --> More_Freight
  if_state2 --> Stop
  More_Freight --> Stop
  Stop --> [*]
  ```
  