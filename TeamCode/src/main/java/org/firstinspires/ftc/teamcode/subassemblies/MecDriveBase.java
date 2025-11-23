package org.firstinspires.ftc.teamcode.subassemblies;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.MathEx;
import org.firstinspires.ftc.teamcode.util.Subassembly;

public class MecDriveBase extends Subassembly {

    public DcMotor leftFront, rightFront, leftRear, rightRear;

    public DcMotor[] motors;

    public MecDriveBase(OpMode opMode) {
        super(opMode, "Mecanum Drive Base");

        leftFront = hardwareMap.dcMotor.get("left_front");
        rightFront = hardwareMap.dcMotor.get("right_front");
        leftRear = hardwareMap.dcMotor.get("left_rear");
        rightRear = hardwareMap.dcMotor.get("right_rear");

        motors = new DcMotor[]{ leftFront, rightFront, leftRear, rightRear };

//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Control the robot with a gamepad, usually called from a TeleOp
     *
     * @param gamepad the gamepad used to move the driveBase, usually gamepad1
     */
    public void control(Gamepad gamepad) {
        double leftX = MathEx.powerCurve(gamepad.left_stick_x);
        double leftY = MathEx.powerCurve(-gamepad.left_stick_y);
        double rightX = gamepad.right_stick_x;

        moveRobot(leftX, leftY, rightX);
    }

    /**
     * Moves the robot based on vectors between -1 and 1
     *
     * @param x strafe power
     * @param y forward/back power
     * @param yaw rotational power
     */
    public void moveRobot(double x, double y, double yaw) {
        // from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double denominator = max(abs(y) + abs(x) + abs(yaw), 1.0);
        double leftFrontPower = (y + x + yaw) / denominator;
        double rightFrontPower = (y - x - yaw) / denominator;
        double leftRearPower = (y - x + yaw) / denominator;
        double rightRearPower = (y + x - yaw) / denominator;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void stopMotors() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }
}