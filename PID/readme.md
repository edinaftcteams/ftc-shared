# Proportional Integral Derivative

There is a lot of reading with a bit of code, but not a lot of "copy and
paste" code. Think of this as a place to learn what PID is and how you
might use it.

P = Proportional

I - Integral

D - Derivative

Some FTC applications for PID controllers:

- Motor Control - This is already in most newer motors. Some teams think
  they need to do it themselves, which may have been true for older
  motors, but the consensus seems to be that for general motor
  applications the latest Rev Robotics and HiTech motors are pretty
  good. The DcMotorEx class in the FTC sdk does allow you to tune the
  PID algorithm. You should at least understand what PID is used for so
  you can decide if tuning it would help your robot.

- Navigation - This is a requirement for autonomous in most games. A
  gyro determines your heading and a PID controller can manage your
  steering.

- Arm Movement - You can make arm movements more accurately accomplish
  their tasks using PID controllers to manage speed and power as they
  move to different positions.

Here are some links for more reading pleasure:

- [Game Manual 0](https://gm0.org/en/stable/docs/software/control-loops.html)
  \- PID explanations and application to FTC.
- [PID](PIDExplained.md) - The details of how PID works, including code
  examples.
- [PID Tutorial](PIDtutorial.pdf) - A 16 page pdf document with a good
  explanation of PID.
- [Beginners PID](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)
  \- This one is for Arduino, but the concepts translate to any
  language.
