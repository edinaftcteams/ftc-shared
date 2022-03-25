# Arm Gravity Compensation

The following is from a forum discussion about the issues of raising and
lowering arms using a motor. The forum discussion is no longer there (it
was a couple of years back.)

Controlling an arm is tricky. It is even trickier if the arm can change
in length (i.e. extend and retract). In general, a fixed length swing
arm going up is working much harder because it is against gravity and
coming down will be extremely fast with gravity helping. So it is not
easy to control an arm in a stable manner. One simple way to do it is to
determine two different power values, one for going up which has a much
larger magnitude than the value used for coming down. This is relatively
simple but it is still not working very well because the load of the arm
is not constant. The load is minimum when the arm is in vertical
position and maximum when it is completely horizontal. And the load
varies with the cosine of the arm angle. A more sophisticated way is to
do something called gravity compensation. Assume the arm is in outer
space in zero gravity, the movement of the arm will be linearly
proportional to the motor power. So if you can manage to add a
compensation factor that makes the arm looks as if it is operating in
outer space, then you can just control it with linear power value.

```java
public void setArmPower(double power)
{
    armMotor.setPower(Range.clip(power + gravityCompensation, -1.0, 1.0));
}
```

gravityCompensation will be calculated according to the arm angle.
Therefore, gravityCompensation is changing while the arm is moving. In
theory, if you call setArmPower(0.0), the above method will feed a
constant gravityCompensation power to the arm motor such that the arm
will be held stationary at a certain angle. So how do you dynamically
calculate gravityCompensation? Basically, the code looks like this:

```java
gravityCompensation = Math.cos(Math.toRadians(armMotor.getAngle())*ARM_MAX_LOAD_POWER);
```

where ARM_MAX_LOAD_POWER is the power value that makes the arm
stationary at horizontal position. You can determine this value
empirically or you can calculate it with the values obtained from the
motor spec (Stalled torque, no load RPM, etc.) and apply some physics
you learn in school (i.e. the maximum torque at the elbow of the arm is
the weight of the arm at the "tip" multiplied by the length of the arm
when the arm is horizontal). Calculating it will give you a good initial
value and then you can empirically tune the compensation power.
