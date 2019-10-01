# Sensor Fusion

Combining two sensor outputs that should theoretically give you the same information is called "sensor fusion".

The easiest way to go about this is with a tool known as a complimentary filter. The complimentary filter uses a "blend" value, between 0 and 1, to produce the output:
filterOutput = blend*sensor1 + (1-blend)*sensor2;

Typically you have the sensor that is more reliable short term weighted much more than the sensor that is more reliable long term. Blend values around 0.95 are common. If you think both sensors are equally reliable, then set blend to 0.5. This simply averages the two.

In your case you know odometry is unreliable long term because of wheel slip and IMU short term because of delays, so I would weight odometry 0.95 and IMU 0.05 to start. See how that works and adjust as necessary. 

