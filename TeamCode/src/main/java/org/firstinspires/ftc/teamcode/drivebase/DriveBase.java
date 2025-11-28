package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Conversions;

import java.util.Arrays;
import java.util.Hashtable;

/**
 * To create a new DriveBase from this pattern, extend this class,
 * define each motor from the hardware map, and
 * Override initMotorConfigurations.
 * Be sure to set the wheelDiameterInches and wheelBaseWidth to suit.
 * You may also wish to adjust the power settings by Overriding initDriveSpeedConfigurations.
 * When using the driveBase in your OpMode, declare a variable of the appropriate DriveBase subclass type
 * in your OpMode class and assign a new instance of that subclass, passing in the hardware map from the OpMode.
 */
public abstract class DriveBase {
    protected HardwareMap hardwareMap;

    /**
     * When instantiating a DriveBase subclass, pass in the hardware map from the OpMode.
     * @param hwMap the hardware map from the opmode
     */
    public DriveBase(HardwareMap hwMap) {
        hardwareMap = hwMap;
        initMotors();
    }
    /**
     * Constants to represent drive speeds that are useful.  Set power values for each in initDriveSpeedConfiguration().
     */
    public enum DriveSpeed {
        STOP,
        SLOW,
        FAST,
        MAX
    }

    // Wheel diameter tells us how far it will travel.
    public double wheelDiameterInches = Conversions.mmToInches(100);
    public double wheelRotationDistanceInches = Math.PI * wheelDiameterInches;
    public double motorRotationsPerInch = 1 / wheelRotationDistanceInches;
    public double wheelBaseWidth = 16; // inches
    // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-223-rpm-3-3-5v-encoder/
    public double motorEncoderEventsPerRotation = 753.2;
    public double motorEncoderEventsPerInch = motorRotationsPerInch * motorEncoderEventsPerRotation;

    public double maxPower = 1; // range of 0 to 1

    /**
     * A predefined set of travel directions. Set motor configurations for each direction in initMotorConfigurations().
     */
    public enum TravelDirection {
        strafeLeft,
        strafeLeftForward,
        strafeLeftBackward,

        strafeRight,
        strafeRightForward,
        strafeRightBackward,

        pivotLeft,
        pivotRight,
        forward,
        reverse
    }


    protected Hashtable<TravelDirection, DcMotorSimple.Direction[]> motorConfigurations = new Hashtable<>();
    protected DcMotor[] wheelMotors = new DcMotor[]{};

    /**
     * Add hashtable entries to Hashtable&lt;TravelDirection, DcMotorSimple.Direction[]&gt; motorConfigurations,
     * representing the directions each motor must turn to send the bot in a given direction.
     * This is mostly useful for autonomous driving; however, teleOp may wish to use these to understand
     * motor directions when handling user input from a gamepad.
     */
    protected void initMotorConfigurations() {
        // define a series of motors for the drivebase here.
        // ex: wheelMotors = new DcMotor[] { leftFront, leftRear, rightRear, rightFront };

        // add hashtable entries to the motorConfigurations here, representing the directions each motor must turn
        // in order for the bot to travel in a given direction.
        // ex: motorConfigurations.put(TravelDirection.forward, Direction.FORWARD);
        //

    }

    /**
     * Returns the defined array of directions for motors, as set in the initMotorConfigurations method.
     * @param travelDirection which direction the bot should travel
     * @return the defined motor directions, or null if no matching configuration has been defined.
     */
    public DcMotorSimple.Direction[] getMotorConfigurations(TravelDirection travelDirection) {
        return motorConfigurations.getOrDefault(travelDirection, null);
    }

    protected Hashtable<DriveSpeed, Double> driveSpeedConfigurations = new Hashtable<>();

    /**
     * Set up the standard power constants for the different drive speeds.
     */
    protected void initDriveSpeedConfigurations()  {
        // add hashtable entries to the driveSpeedConfigurations here, based on the configured motors and wheels.
        driveSpeedConfigurations.put(DriveSpeed.STOP, 0.0);
        driveSpeedConfigurations.put(DriveSpeed.SLOW, 0.2);
        driveSpeedConfigurations.put(DriveSpeed.FAST, 0.8);
        driveSpeedConfigurations.put(DriveSpeed.MAX, 1.0);
    }

    /**
     * Returns the power value for the given drive speed, or null.
     * @param driveSpeed how fast we should go - use the enum
     * @return the current power for the drive speed
     */
    public double getDriveSpeedPower(DriveSpeed driveSpeed) {
        return driveSpeedConfigurations.getOrDefault(driveSpeed, (double) 0);
    }

    /**
     * Configure motors in initMotorConfigurations.
     * @return the series, in order as configured, of motor objects that are relevant to this drivebase.
     */
    public DcMotor[] getMotors() {
        return wheelMotors;
    }

    /**
     * Initialize all motors to run using encoders.
     * Set travel to forward and power to  0.
     */
    public void initMotors() {

        initMotorConfigurations();
        initDriveSpeedConfigurations();

        // Output all configuration information for the drive base.
        RobotLog.i("*******  Drive Base Configuration *******");
        RobotLog.i("Wheel diameter (inches): " + String.valueOf(wheelDiameterInches));
        RobotLog.i("Floor distance for one wheel rotation (inches): " + String.valueOf(wheelRotationDistanceInches));
        RobotLog.i("Rotations per inch of floor distance: " + String.valueOf(motorRotationsPerInch));
        RobotLog.i("Wheel base width (inches): " + String.valueOf(wheelBaseWidth));
        RobotLog.i("Motor encoder events per wheel rotation: " + String.valueOf(motorEncoderEventsPerRotation));
        RobotLog.i("Motor encoder events per inch of floor distance: " + String.valueOf(motorEncoderEventsPerInch));
        RobotLog.i("Maximum power to motors: " + String.valueOf(maxPower));
        RobotLog.i("*******  *******");

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setStopMode(DcMotor.ZeroPowerBehavior.BRAKE);

        setTravelDirection(TravelDirection.forward);

        // set motor power to 0.
        stop();
    }

    /**
     * Remove instances of motors, travel directions, speeds
     */
    public void cleanup() {
        stop();
        motorConfigurations.clear();
        driveSpeedConfigurations.clear();
        // wheelMotors = null; // unnecessary, GC will handle this for us.
    }

    /**
     * Set the runmode on all motors.
     * @param runMode how the motors should run - use the enum.
     * @return boolean success
     */
    public boolean setRunMode(DcMotor.RunMode runMode) {
        try {
            DcMotor[] motors = getMotors();
            for (DcMotor motor : motors) {
                motor.setMode(runMode);
            }
            return true;
        }
        catch (Exception e) {
            RobotLog.i(e.getMessage());
            return false;
        }
    }

    /**
     * Set the zero power behavior on all motors.
     * @param behavior zero power behavior - use the enum
     * @return boolean success
     */
    public boolean setStopMode(DcMotor.ZeroPowerBehavior behavior) {
        try {
            for (DcMotor motor : getMotors()) {
                motor.setZeroPowerBehavior(behavior);
            }
            return true;
        }
        catch (Exception e) {
            RobotLog.i(e.getMessage());
            return false;
        }
    }

    /**
     * Returns the encoder values for all motors.
     * @return current positions of all motors, in order configured.
     */
    public int[] getEncoderPositions() {
        DcMotor[] motors = getMotors();
        int[] positions = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            positions[i] = motors[i].getCurrentPosition();
        }
        return positions;
    }

    /**
     * Set the target position tolerance on each motor.
     * @param tolerance position tolerance
     * @return position tolerance from each motor, in order configured.
     */
    public int[] setPositionTolerance(int tolerance) {
        // set position tolerance
        DcMotor[] motors = getMotors();
        int[] tolerances = new int[] {0, 0, 0, 0};
        for (int i = 0; i < motors.length; i++) {
            ((DcMotorEx) motors[i]).setTargetPositionTolerance(tolerance);
            tolerances[i] = ((DcMotorEx) motors[i]).getTargetPositionTolerance();
        }
        return tolerances;
    }

    /**
     * Returns the target position tolerance of each motor as an integer array.
     * This probably is unnecessary.  We can remove it if so.
     * @return position tolerance from each motor, in order configured.
     */
    public int[] getPositionTolerance() {
        // set position tolerance
        DcMotor[] motors = getMotors();
        int[] tolerances = new int[] {0, 0, 0, 0};
        for (int i = 0; i < motors.length; i++) {
            tolerances[i] = ((DcMotorEx) motors[i]).getTargetPositionTolerance();
        }
        return tolerances;
    }

    /**
     * Set the direction of travel to one of the predefined configurations.
     * @param travelDirection which way should the bot go?
     * @return  success
     */
    public boolean setTravelDirection(TravelDirection travelDirection) {
        // our motor configuration
        try {
            DcMotor[] motors = getMotors();
            DcMotorSimple.Direction[] directions = getMotorConfigurations(travelDirection);
            for (int i = 0; i < motors.length; i++) {
                motors[i].setDirection(directions[i]);
            }
            return true;
        }
        catch (Exception e) {
            stop();
            return false;
        }
    }

    public boolean isBusy() {
        DcMotor[] motors = getMotors();
        for (DcMotor motor : motors) {
            if (motor.isBusy()) { return true; }
        }
        return false;
    }

    /**
     * Stop all motors - set power to 0.
     * @return power
     */
    public double stop()  {
        DcMotor[] motors = getMotors();
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
        return 0;
    }

    /**
     * Go in one of the set directions at the set speed.  Stops on exception.
     * @param direction which way should the bot go?
     * @param driveSpeed how fast?
     * @return the power to the motors
     */
    public double go(TravelDirection direction, DriveSpeed driveSpeed)  {
        double power = getDriveSpeedPower(driveSpeed);
        return go(direction, power);
    }

    /**
     * Go in one of the set directions at the set power. Stops on exception.
     * @param direction  which way should the bot go?
     * @param power power value for the motors
     * @return the power to the motors
     */
    public double go(TravelDirection direction, double power) {
        DcMotorSimple.Direction[] directions = getMotorConfigurations(direction);
        return go(directions, power);
    }

    /**
     * Send the motors in any given direction at the set power.  Stops on exception.
     * @param directions  which way should each motor turn?
     * @param power power value for the motors
     * @return the power to the motors
     */
    public double go(DcMotorSimple.Direction[] directions, double power) {
        try {
            RobotLog.i("Power: " + String.valueOf(power) + " Directions: " + Arrays.toString(directions));
            DcMotor[] motors = getMotors();
            for (int i = 0; i < motors.length; i++) {
                motors[i].setDirection(directions[i]);
                motors[i].setPower(power);
                //motors[i].setPower(directions[i] == null ? 0 : power);
            }
            return power;
        }
        catch (Exception e) {
            RobotLog.i(e.getMessage());
            return stop();
        }
    }

    /**
     * Set power levels on each motor separately.  Allows finer navigational control of travel paths.
     * Useful especially for a game controller interface.
     * @param powerLevels power level for motor, in order as configured.
     * @return the power level of the first motor.
     */
    public double go(double[] powerLevels) {
        DcMotor[] motors = getMotors();

        // if there aren't enough power levels in the array, add 0 on the end.
        if (powerLevels.length < motors.length) {
            stop();
            throw new ArrayIndexOutOfBoundsException("There are more motors than power levels specified.");
        }

        // set each motor in the array to its pair in the power array.
        for (int i  = 0; i < motors.length; i++) {
            motors[i].setPower(powerLevels[i]);
        }

        // return the first motor's power level.
        return powerLevels.length > 0 ? powerLevels[0] : 0;
    }


    /**
     * Drive a number of ticks in a particular direction. Stops on exception.
     * @param travelDirection which way should the bot go?
     * @param power the power to each motor
     * @param encoderTicks the distance each motor  should go, by the encoders
     */
    public double go(TravelDirection travelDirection, double power, int encoderTicks) {
        DcMotorSimple.Direction[] driveConfiguration = getMotorConfigurations(travelDirection);
        int[] wheelEncoderTicks = new int[driveConfiguration.length];
        Arrays.fill(wheelEncoderTicks, encoderTicks);
        return go(driveConfiguration, power, wheelEncoderTicks);
    }

    /**
     * Drive a number of ticks in a particular direction. Stops on exception.
     * @param driveConfiguration is a list of Directions, which represent the motor direction for each wheel.
     * @param power the power to each motor (we can make this a list if  needed)
     * @param encoderTicks a  list of  encoder tick values,  distances  that each motor should go by the encoders.
     */
    public double go(DcMotorSimple.Direction[] driveConfiguration, double power, int[] encoderTicks) {
        int [] currentPositions = getEncoderPositions();

        RobotLog.i("Current Positions when driving ticks: " + Arrays.toString(currentPositions));
        RobotLog.i("Drive Configuration: " + Arrays.toString(driveConfiguration));
        RobotLog.i("Power: " + String.valueOf(power) + " Encoder Ticks: " + Arrays.toString(encoderTicks));

        DcMotor[] motors = getMotors();

        // set each motor, based on drive configuration/travel direction
        for (int i = 0; i < driveConfiguration.length; i++) {
            int targetPosition = (int)calcTargetPosition(driveConfiguration[i], currentPositions[i], encoderTicks[i]);
            RobotLog.i("Target Position for " + motors[i].getDeviceName() + " calculated as " + String.valueOf(targetPosition));
            motors[i].setTargetPosition(targetPosition);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // then set all power
        return go(driveConfiguration, power);
    }

    /**
     * Utility method for use in setting encoder values.
     * @param motorDirection which way should the bot  go?
     * @param currentPosition where are we now?
     * @param encoderTicks how far should we go?
     * @return the encoder value we're shooting for.
     */
    private double calcTargetPosition(DcMotorSimple.Direction motorDirection, double currentPosition, double encoderTicks) {
        switch (motorDirection) {
            case FORWARD:
                return currentPosition + encoderTicks;
            case REVERSE:
                return currentPosition - encoderTicks;
            default:
                return currentPosition;
        }
    }

    /**
     * Returns ticks for the encoder to run when we want to pivot around our own middle.
     * @param theta is the angle in degrees that we should be turning.
     * @return the encoder ticks needed to move the bot that far.
     */
    public double getEncoderValueForRobotPivotAngle(float theta) {
        // calculate the arc of the pivot
        double radius = wheelBaseWidth/2;
        double arcLength = radius * theta * Math.PI / 180; // length in inches

        // now that we have the arcLength, we figure out how many wheel rotations are needed.
        return getEncoderValueForRobotInches(arcLength);
    }

    /**
     * Returns ticks for the encoder to run when we want to cover a distance.
     * @param inchesToTravel how far in inches should the bot go?
     * @return  the encoder ticks needed to move the bot that far.
     */
    public double getEncoderValueForRobotInches(double inchesToTravel) {
        // number of ticks for the encoder
        double ticks = inchesToTravel * motorEncoderEventsPerInch;
        RobotLog.i("Calculated ticks for " + inchesToTravel + " as " + String.valueOf(ticks));
        return ticks;
    }
}
