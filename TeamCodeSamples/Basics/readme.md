# Basics

***Come here first to see how to code your robot.***

The
[external.samples](https://github.com/FIRST-Tech-Challenge/FtcRobotController/tree/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples)
folder in the FTC sdk is the best place to see FTC coding examples.

There are lots of examples in the samples for motors, sensors, cameras
and more. Be aware that some of the samples may not be applicable to the
specific hardware you are using.

## Op Modes

An understanding of op modes is a requirement for creating a successful
robot control app. The first thing to understand is the difference
between iterative and linear op modes. Linear is easier to understand
but iterative can give you more control. Either style will work. The
choice is made by you based on your robot and its coding requirements.

The following op modes are located in the FTC sdk in this folder:
[external.samples](https://github.com/FIRST-Tech-Challenge/FtcRobotController/tree/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples)

1. BasicOpMode_Linear - This is a simple 2 motor linear teleop op mode.
   Blocks creates linear op modes by default. Look at the next op mode
   to see an example of iterative op modes.
2. BasicOpMode_Iterative - The sdk platform loops automatically for you.
   This is an example of an *event driven* programming model.
3. BasicOmniOpMode_Linear - You must write the loop in your code.

### Holonomic Drive Train

Given the interest in mecanum wheels and holonomic drive trains,
[EdinaFTCOmniTest.java](EdinaFTCOmniTest.java) in ths folder, is a great
place to begin your understanding and testing of holonomic drives. This op mode
should be a permanent part of your test toolkit for verifying your
holonomic drive train setup.
