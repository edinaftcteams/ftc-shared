# Motor Encoder Ticks

This one comes up all the time when creating your functions for driving so you can use inches instead of encoder ticks. 

Looking up the motor encoder specs should give you the counts per revolution for your motors. And in theory you can combine this number with your gear reduction and wheel size to predict how many counts per inch you will get (this is a great constant value to use in your program....)
You can also get this value in code if your robot configuration has the correct motor defined.

```java
double COUNTS_PER_MOTOR_REV;

    COUNTS_PER_MOTOR_REV = motorLeft.getMotorType().getTicksPerRev();
```

But.... after having done this calculation, you should then verify the number with some experimentation (which can also help if you don't have motor specs).

Write an opmode to repeatedly call myMotor.getCurrentPosition() and display it as a telemetry value.

Set up a test where you can compare counts with inches. There a LOTS of ways to do this (it's easier if you reset the encoders before each test, but it's not required.)

You can:

1. Manually rotate the wheel 10 times. Take the difference between the starting and ending encoder value. Divide the change in encoder value by (10 * D * PI).

or

2. Change the program to drive for 10 seconds and then stop. Take the difference between the starting and ending encoder value. Divide by the distance traveled.

or

3. Reset the encoder and then drive to 10,000 encoder counts and stop. Divide 10,000 by the distance traveled.

In all cases drive the motors slowly so you don't get slippage, and use a long time or distance to improve your accuracy. 
