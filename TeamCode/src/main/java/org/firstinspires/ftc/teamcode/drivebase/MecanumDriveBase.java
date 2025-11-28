package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Use this superclass to control a drive base with four mecanum wheels, one at each corner.
 */
public class MecanumDriveBase extends DriveBase {

    // @Device("left_front")
    public DcMotor motorLeftFront;

    // @Device("left_rear")
    public DcMotor motorLeftRear;

    // @Device("right_rear")
    public DcMotor motorRightRear;

    // @Device("right_front")
    public DcMotor motorRightFront;

    public MecanumDriveBase(HardwareMap hwMap) {
        super(hwMap);
    }

    @Override
    protected void initMotorConfigurations() {
        motorLeftFront  = hardwareMap.dcMotor.get("left_front");
        motorRightFront  = hardwareMap.dcMotor.get("right_front");
        motorLeftRear  = hardwareMap.dcMotor.get("left_rear");
        motorRightRear  = hardwareMap.dcMotor.get("right_rear");
        
        wheelMotors = new DcMotor[]{ motorLeftFront, motorLeftRear, motorRightRear, motorRightFront };

        motorConfigurations.put(TravelDirection.strafeLeftForward, new Direction[]{null, leftMotorDirection(Direction.FORWARD), null, rightMotorDirection(Direction.FORWARD)});
        motorConfigurations.put(TravelDirection.strafeLeft, new Direction[]{leftMotorDirection(Direction.REVERSE), leftMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.REVERSE), rightMotorDirection(Direction.FORWARD)});
        motorConfigurations.put(TravelDirection.strafeLeftBackward, new Direction[]{leftMotorDirection(Direction.REVERSE), null, rightMotorDirection(Direction.REVERSE), null});

        motorConfigurations.put(TravelDirection.strafeRightForward, new Direction[]{leftMotorDirection(Direction.FORWARD), null, rightMotorDirection(Direction.FORWARD), null});
        motorConfigurations.put(TravelDirection.strafeRight, new Direction[]{leftMotorDirection(Direction.FORWARD), leftMotorDirection(Direction.REVERSE), rightMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.REVERSE)});
        motorConfigurations.put(TravelDirection.strafeRightBackward, new Direction[]{null, leftMotorDirection(Direction.REVERSE), null, rightMotorDirection(Direction.REVERSE)});

        motorConfigurations.put(TravelDirection.forward, new Direction[]{leftMotorDirection(Direction.FORWARD), leftMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.FORWARD)});
        motorConfigurations.put(TravelDirection.reverse, new Direction[]{leftMotorDirection(Direction.REVERSE), leftMotorDirection(Direction.REVERSE), rightMotorDirection(Direction.REVERSE), rightMotorDirection(Direction.REVERSE)});

        motorConfigurations.put(TravelDirection.pivotLeft, new Direction[]{leftMotorDirection(Direction.REVERSE), leftMotorDirection(Direction.REVERSE), rightMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.FORWARD)});
        motorConfigurations.put(TravelDirection.pivotRight, new Direction[]{leftMotorDirection(Direction.FORWARD), leftMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.REVERSE), rightMotorDirection(Direction.REVERSE)});

        // super.initMotorConfigurations();
    }
    // for consistency, always use these methods to translate the actual motor direction
    // for the different sides of the bot.
    // if the motor orientation changes on the hardware, adjust these methods to suit.
    private static Direction rightMotorDirection(Direction externalDirection) {
        return externalDirection;
    }
    // reverse the motor direction for the motors on the right side of the bot.
    private static Direction leftMotorDirection(Direction externalDirection) {
        if (externalDirection == Direction.FORWARD) {
            return Direction.REVERSE;
        } else {
            return Direction.FORWARD;
        }
    }

}

