package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Ron on 11/19/2019.
 * Autonomous using gyro for navigation.
 */
//------------------------------------------------------------------------------
// Extends the OpMode class to provide Autonomous code
//------------------------------------------------------------------------------
/* This opMode does the following steps:
 * 0) Wait till the encoders show reset to zero.
 * 1) Drives to the vicinity of the beacon using encoder counts
 * 2) Pushes up against wall to get square using constant power an time.
 * 3) Deploys the Climbers using the arm motor
 * 4) Drives to the Mountain using encoder counts
 * 5) Climbs the Mountain using constant speed and time
 * 6) Stops and waits for end of Auto
 *
 * Uses gyro to correct heading and for turn angles.
 * The code is executed as a state machine.  Each "State" performs a specific task which takes time to execute.
 * An "Event" can cause a change in the state.  One or more "Actions" are performed when moving on to next state
 */
@Autonomous(name = "State Machine Gyro", group = "Iterative")
//@Disabled
public class RHSAutoStateMachineGyro extends OpMode {
    AutonomousConfiguration ftcConfig = new AutonomousConfiguration(gamepad1, telemetry);

    // A list of system States.
    private enum State {
        STATE_INITIAL,
        STATE_NAVIGATE_TO_SKY_BRIDGE,
        STATE_SQUARE_TO_WALL,
        STATE_BACKUP,
        STATE_DEPLOY_CLIMBERS,
        STATE_DRIVE_TO_MOUNTAIN,
        STATE_CLIMB_MOUNTAIN,
        STATE_STOP,
    }

    private enum NavigateState {
        INITIAL,
        NAVIGATE_TO_SKYBRIDGE
    }

    private enum RepositionState {
        INITIAL,
        DRIVE_TO_FOUNDATION,
        POSITION_TO_BUILDING_SITE,
        NAVIGATE_TO_SKYBRIDGE
    }

    /*
       Use the gyro to map paths.
       A segment is a heading, speed and distance. When speed is 0 we are turning.
     */
    private final GyroPathSeg[] deliverFindPath = {
            new GyroPathSeg(0, .6, -35),
            new GyroPathSeg(45, 0, 0),
            new GyroPathSeg(45, .6, -60),
            new GyroPathSeg(90, 0, 0),
            new GyroPathSeg(90, .6, -30),
    };

    final GyroPathSeg[] mGyroMountainPath = {
            new GyroPathSeg(90, .6, 35),
            new GyroPathSeg(45, 0, 0),
            new GyroPathSeg(45, .6, 15),
            new GyroPathSeg(-45, 0, 0),
    };

    final double COUNTS_PER_INCH = 160;    // Number of encoder counts per inch of wheel travel.
    final double COUNTS_PER_DEGREE = .18;  // Counts to rotate 1 degree.

    // Encoder counts to move arm to and from climber drop.
    final int CLIMBER_DEPLOY = -1410; // Switch sign to retract
    boolean mClimberDeploying = true;

    // PID control values for heading control using the gyro.
    final double Kp = .8;
    final double Ki = .1;
    int mHeadingError = 0;
    double mIntegral = 0;
    double mDt = 0;

    //--------------------------------------------------------------------------
    // Robot device Objects
    //--------------------------------------------------------------------------
    public DcMotor mLeftMotor;
    public DcMotor mRightMotor;
    public DcMotor mLeftArm;
    public GyroSensor mGyro;
    //public LightSensor mLight;
    //public OpticalDistanceSensor mDistance;

    private int mLeftEncoderTarget;
    private int mRightEncoderTarget;
    private int mCurrentArmTarget;
    private int mCurrentHeadingTarget;

    // Loop cycle time stats variables
    public ElapsedTime mRuntime = new ElapsedTime();   // Time into round.

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    private State mCurrentState;    // Current State Machine State.
    private GyroPathSeg[] mCurrentGyroPath; // Array holds current gyro path.
    private int mCurrentSeg;      // Index of the current leg in the current path


    //--------------------------------------------------------------------------
    // Constructor
    //--------------------------------------------------------------------------
    public RHSAutoStateMachineGyro() {
    }

    //--------------------------------------------------------------------------
    // init
    //--------------------------------------------------------------------------
    @Override
    public void init() {
        // Initialize class members.
        mLeftMotor = hardwareMap.dcMotor.get("left_drive");
        mRightMotor = hardwareMap.dcMotor.get("right_drive");
        mRightMotor.setDirection(DcMotor.Direction.REVERSE);
        setDrivePower(0, 0);        // Ensure motors are off
        resetDriveEncoders();       // Reset Encoders to Zero
        mLeftArm = hardwareMap.dcMotor.get("left_arm");
        mGyro = hardwareMap.gyroSensor.get("gyro");
        mGyro.calibrate();
        // Assume start position is "rest" position.
        setArmMode(DcMotorController.RunMode.RESET_ENCODERS);
        setArmMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        ftcConfig.ShowMenu();
    }

    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    // @Override
    public void init_loop() {
    }

    //--------------------------------------------------------------------------
    // start
    //--------------------------------------------------------------------------
    @Override
    public void start() {
        // Setup Robot devices, set initial state and start game clock
        setDriveSpeed(0, 0);        // Set target speed to zero
        runToPosition();            // Run to Position set by encoder targets
        mRuntime.reset();           // Zero game clock
        newState(State.STATE_INITIAL);
    }

    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    @Override
    public void loop() {
        // Send the current state info (state and time) back to first line of driver station telemetry.
        telemetry.addData("0", String.format("%4.1f ", mStateTime.time()) + mCurrentState.toString());

        // Execute the current state.  Each STATE's case code does the following:
        // 1: Look for an EVENT that will cause a STATE change
        // 2: If an EVENT is found, take any required ACTION, and then set the next STATE
        //   else
        // 3: If no EVENT is found, do processing for the current STATE and send TELEMETRY data for STATE.
        //
        switch (mCurrentState) {
            case STATE_INITIAL:         // Stay in this state until encoders are both Zero.
                if (encodersAtZero() &&
                        (mStateTime.time() >= delayStartSeconds) &&
                        (!mGyro.isCalibrating())) {
                    startPath(mGyroBeaconPath);                 // Action: Load path to beacon
                    newState(State.STATE_DRIVE_TO_BEACON);  // Next State:
                } else {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("L %5d - R %5d ", getLeftPosition(),
                            getRightPosition()));
                }
                break;

            case STATE_DRIVE_TO_BEACON: // Follow path until last segment is completed
                if (pathComplete() || mStateTime.time() > 10) {
                    setDriveSpeed(-0.2, -0.2);                // Action: Drive a little more
                    newState(State.STATE_SQUARE_TO_WALL);      // Next State:
                } else {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("%d of %d. L %5d:%5d - R %5d:%5d ",
                            mCurrentSeg, mCurrentGyroPath.length,
                            mLeftEncoderTarget, getLeftPosition(),
                            mRightEncoderTarget, getRightPosition()));
                }
                break;

            case STATE_SQUARE_TO_WALL:     // Push up against wall
                if (mStateTime.time() > 1) {
                    setDriveSpeed(0.2, 0.2);                // Action: Backup
                    newState(State.STATE_BACKUP);  // Next State:
                }
                break;

            case STATE_BACKUP:
                if (mStateTime.time() > 1) {
                    setDriveSpeed(0.0, 0.0);
                    startDeployArm();                       // Start arm to deploy climbers
                    newState(State.STATE_DEPLOY_CLIMBERS);  // Next State:
                }
                break;

            case STATE_DEPLOY_CLIMBERS:     // wait while arm moves and deposits climbers
                if ((mStateTime.time() > 3) || (armDeployComplete())) {
                    mLeftArm.setPower(0);
                    // Skip mountain if configured to beacon.
                    if (ftcConfig.param.autonType == FtcConfig.AutonType.GO_FOR_BEACON) {
                        setDrivePower(0, 0);
                        newState(State.STATE_STOP);
                    } else {
                        startPath(mGyroMountainPath);               // Action: Load path to Mountain
                        newState(State.STATE_DRIVE_TO_MOUNTAIN);// Next State:
                    }
                }
                break;

            case STATE_DRIVE_TO_MOUNTAIN: // Follow path until last segment is completed
                if (pathComplete() || mStateTime.time() > 5) {
                    useConstantPower();                     // Action: Switch to constant Speed
                    setDrivePower(0.6, 0.6);                // Action: Start Driving forward
                    newState(State.STATE_CLIMB_MOUNTAIN);   // Next State:
                } else {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("%d of %d. L %5d:%5d - R %5d:%5d ",
                            mCurrentSeg, mCurrentGyroPath.length,
                            mLeftEncoderTarget, getLeftPosition(),
                            mRightEncoderTarget, getRightPosition()));
                }
                break;

            case STATE_CLIMB_MOUNTAIN:   // Drive up mountain
                if (mStateTime.time() > 3) {
                    setDrivePower(0, 0);                    // Set target speed to zero
                    newState(State.STATE_STOP);             // Next State:
                } else {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("L %5d - R %5d ", getLeftPosition(),
                            getRightPosition()));
                }
                break;

            case STATE_STOP:
                break;
        }
    }

    //--------------------------------------------------------------------------
    // stop
    //--------------------------------------------------------------------------
    @Override
    public void stop() {
        // Ensure that the motors are turned off.
        useConstantPower();
        setDrivePower(0, 0);
    }

    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //  Transition to a new state.
    //--------------------------------------------------------------------------
    private void newState(State newState) {
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentState = newState;
    }


    //--------------------------------------------------------------------------
    // setEncoderTarget( LeftEncoder, RightEncoder);
    // Sets Absolute Encoder Position
    //--------------------------------------------------------------------------
    void setEncoderTarget(int leftEncoder, int rightEncoder) {
        // Reverse turns if we are red alliance.
        if (leftEncoder != rightEncoder) {
            if (ftcConfig.param.colorIsRed) {
                leftEncoder = leftEncoder * -1;
                rightEncoder = rightEncoder * -1;
            }
        }

        mLeftMotor.setTargetPosition(mLeftEncoderTarget = leftEncoder);
        mRightMotor.setTargetPosition(mRightEncoderTarget = rightEncoder);
    }

    //--------------------------------------------------------------------------
    // addEncoderTarget( LeftEncoder, RightEncoder);
    // Sets relative Encoder Position.  Offset current targets with passed data
    //--------------------------------------------------------------------------
    void addEncoderTarget(int leftEncoder, int rightEncoder) {
        // Reverse turns if we are red alliance.
        if (leftEncoder != rightEncoder) {
            if (ftcConfig.param.colorIsRed) {
                leftEncoder = leftEncoder * -1;
                rightEncoder = rightEncoder * -1;
            }
        }

        mLeftMotor.setTargetPosition(mLeftEncoderTarget += leftEncoder);
        mRightMotor.setTargetPosition(mRightEncoderTarget += rightEncoder);
    }

    //--------------------------------------------------------------------------
    // setDrivePower( LeftPower, RightPower);
    //--------------------------------------------------------------------------
    void setDrivePower(double leftPower, double rightPower) {
        mLeftMotor.setPower(Range.clip(leftPower, -1, 1));
        mRightMotor.setPower(Range.clip(rightPower, -1, 1));
    }

    //--------------------------------------------------------------------------
    // setDriveSpeed( LeftSpeed, RightSpeed);
    //--------------------------------------------------------------------------
    void setDriveSpeed(double leftSpeed, double rightSpeed) {
        setDrivePower(leftSpeed, rightSpeed);
    }

    //--------------------------------------------------------------------------
    // runToPosition ()
    // Set both drive motors to encoder servo mode (requires encoders)
    //--------------------------------------------------------------------------
    public void runToPosition() {
        setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    //--------------------------------------------------------------------------
    // useConstantSpeed ()
    // Set both drive motors to constant speed (requires encoders)
    //--------------------------------------------------------------------------
    public void useConstantSpeed() {
        setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // useConstantPower ()
    // Set both drive motors to constant power (encoders NOT required)
    //--------------------------------------------------------------------------
    public void useConstantPower() {
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // resetDriveEncoders()
    // Reset both drive motor encoders, and clear current encoder targets.
    //--------------------------------------------------------------------------
    public void resetDriveEncoders() {
        setEncoderTarget(0, 0);
        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public void resetLeftArmEncoder() {
        mLeftArm.setTargetPosition(0);
        setArmMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    //--------------------------------------------------------------------------
    // syncEncoders()
    // Load the current encoder values into the Target Values
    // Essentially synch's the software with the hardware
    //--------------------------------------------------------------------------
    void synchEncoders() {
        //	get and set the encoder targets
        mLeftEncoderTarget = mLeftMotor.getCurrentPosition();
        mRightEncoderTarget = mRightMotor.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // setDriveMode ()
    // Set both drive motors to new mode if they need changing.
    //--------------------------------------------------------------------------
    public void setDriveMode(DcMotorController.RunMode mode) {
        // Ensure the motors are in the correct mode.
        if (mLeftMotor.getMode() != mode)
            mLeftMotor.setMode(mode);

        if (mRightMotor.getMode() != mode)
            mRightMotor.setMode(mode);
    }

    public void setArmMode(DcMotorController.RunMode mode) {
        if (mLeftArm.getMode() != mode)
            mLeftArm.setMode(mode);
    }

    //--------------------------------------------------------------------------
    // getLeftPosition ()
    // Return Left Encoder count
    //--------------------------------------------------------------------------
    int getLeftPosition() {
        return mLeftMotor.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // getRightPosition ()
    // Return Right Encoder count
    //--------------------------------------------------------------------------
    int getRightPosition() {
        return mRightMotor.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // moveComplete()
    // Return true if motors have both reached the desired encoder target
    //--------------------------------------------------------------------------
    boolean moveComplete() {
        return ((Math.abs(getLeftPosition() - mLeftEncoderTarget) < 10) &&
                (Math.abs(getRightPosition() - mRightEncoderTarget) < 10));
    }

    // Return true if arm encoder is at target.
    boolean armMoveComplete(int target) {
        return (Math.abs(mLeftArm.getCurrentPosition() - target) < 10);
    }

    //--------------------------------------------------------------------------
    // encodersAtZero()
    // Return true if both encoders read zero (or close)
    //--------------------------------------------------------------------------
    boolean encodersAtZero() {
        return ((Math.abs(getLeftPosition()) < 5) && (Math.abs(getRightPosition()) < 5));
    }

    // Call this to start arm deploy to dump climber.
    // Then call armDeployComplete() until it returns true, signaling arm has
    // retracted.
    private void startDeployArm() {
        resetLeftArmEncoder();
        setArmMode(DcMotorController.RunMode.RUN_TO_POSITION);
        mClimberDeploying = true;
        mCurrentArmTarget = CLIMBER_DEPLOY;
        mLeftArm.setTargetPosition(mCurrentArmTarget);     // Action:  Start deploying climbers.
        mLeftArm.setPower(.3);
    }

    // Returns true when arm has deployed and retracted.
    private boolean armDeployComplete() {
        if (!armMoveComplete(mCurrentArmTarget)) {
            // Did not reach target yet.
            return false;
        }

        mLeftArm.setPower(0);
        if (!mClimberDeploying) {
            // We finished retracting the arm.
            return true;
        }

        // Deploy finished, set to retract.
        mClimberDeploying = false;
        mCurrentArmTarget = -CLIMBER_DEPLOY;
        mLeftArm.setTargetPosition(mCurrentArmTarget);  // Action:  retract arm.
        mLeftArm.setPower(.2);
        return false;
    }

    /*
        Begin the first leg of the path array that is passed in.
        Calls startSeg() to actually load the encoder targets.
     */
    private void startPath(GyroPathSeg[] path) {
        mCurrentGyroPath = path;    // Initialize path array
        mCurrentSeg = 0;
        synchEncoders();        // Lock in the current position
        runToPosition();        // Enable RunToPosition mode
        startSeg();             // Execute the current (first) Leg
    }

    /*
        Starts the current leg of the current path.
        Must call startPath() once before calling this
        Each leg adds the new relative movement onto the running encoder totals.
        By not reading and using the actual encoder values, this avoids accumulating errors.
        Increments the leg number after loading the current encoder targets
     */
    private void startSeg() {
        int Left;
        int Right;
        double speed;

        if (mCurrentGyroPath != null) {
            // Load up the next motion based on the current segment.
            mHeadingError = 0;
            mIntegral = 0;
            speed = mCurrentGyroPath[mCurrentSeg].Speed;
            mCurrentHeadingTarget = mCurrentGyroPath[mCurrentSeg].Heading;

            if (speed == 0) { // We are turning.
                Left = (int) (mCurrentGyroPath[mCurrentSeg].Heading * COUNTS_PER_DEGREE);
                Right = -Left;
                addEncoderTarget(Left, Right);
                setDriveSpeed(.4, .4);
            } else {
                Left = (int) (mCurrentGyroPath[mCurrentSeg].Distance * COUNTS_PER_INCH);
                Right = Left;
                addEncoderTarget(Left, Right);
                setDriveSpeed(mCurrentGyroPath[mCurrentSeg].Speed, mCurrentGyroPath[mCurrentSeg].Speed);
            }

            mCurrentSeg++;  // Move index to next segment of path
        }
    }

    /*
        Determines if the current path is complete
        As each segment completes, the next segment is started unless there are no more.
        Returns true if the last leg has completed and the robot is stopped.
     */
    private boolean pathComplete() {
        // Wait for this Segment to end and then see what's next.
        if (moveComplete()) {
            // Start next Segment if there is one.
            if (mCurrentSeg < mCurrentGyroPath.length) {
                startSeg();
                return false;
            } else  // Otherwise, stop and return done
            {
                mCurrentGyroPath = null;
                mCurrentSeg = 0;
                setDriveSpeed(0, 0);
                useConstantSpeed();
                return true;
            }
        }

        AdjustHeading();
        return false;
    }

    // Return -100 to 100, PID value needed to adjust to mCurrentHeadingTarget.
    private int GetHeadingAdjustment() {
        int adjustment;
        int currentHeading = mGyro.getHeading();
        // Keep heading in range of -179 to 180 (negative west of 0, positive east of 0)
        currentHeading = currentHeading > 180 ? currentHeading - 360 : currentHeading;
        mHeadingError = mCurrentHeadingTarget - currentHeading;
        adjustment = (int) ((Kp * mHeadingError) + (Ki * mIntegral));
        if (adjustment > 100) {
            adjustment = 100;
        } else if (adjustment < -100) {
            adjustment = -100;
        }

        mIntegral += mHeadingError * mDt;
        return adjustment;
    }

    private void AdjustHeading() {
        int adjustment = GetHeadingAdjustment();
        addEncoderTarget((adjustment), (-adjustment));
    }
}

/*
    Segments of robot path using gyro to navigate.
    If Distance is 0 we are only turning.
 */
class GyroPathSeg {
    public int Heading;
    public double Speed;
    public int Distance;

    public GyroPathSeg(int heading, double speed, int distance) {
        Heading = heading;
        Speed = speed;
        Distance = distance;
    }
}
