# PID

This section is a collection of PID information. There is a lot of text and a bit of code, but not a lot of "copy and paste" code. Think of this as a place to learn what PID is and how you might use it.

P = Proportional

I - Integral

D - Derivative

Some FTC applications for PID controllers:

* Motor Control
This is already in most newer motors. Some teams think they need to do it themselves, which may have been true for older motors, but the consensus seems to be that the latest Rev Robotics and HiTech motors are pretty good.

* Navigation
This is a requirement for autonomous in most games. A gyro determines your heading and the PID controller manages your steering.

* Arm Movement
You can make arm movements more accurately accomplish their tasks using PID controllers to manage speed and power as they move to different positions.

Here are some links for more reading pleasure:

* [Beginners PID](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/) - This one is for Arduino, but the concepts translate to any language.
* [Forum Discussion](https://ftcforum.usfirst.org/forum/ftc-technology/4019-high-accuracy-pid-controller-in-android) - Some "techie" information on PID's in FTC.
