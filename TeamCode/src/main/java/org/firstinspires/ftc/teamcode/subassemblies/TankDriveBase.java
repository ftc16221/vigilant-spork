package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.MathEx;
import org.firstinspires.ftc.teamcode.util.Subassembly;

public class TankDriveBase extends Subassembly {

    DcMotor left, right;

    DcMotor[] motors;

    public TankDriveBase(OpMode opMode) {
        super (opMode, "Tank Drive Base");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        motors = new DcMotor[]{ left, right };

//        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void control(Gamepad gamepad) {
        double leftY = MathEx.powerCurve(-gamepad.left_stick_y);
        double rightX = gamepad.right_stick_x;

        // calculate motor powers
        double leftPower = leftY + rightX;
        double rightPower = leftY - rightX;

        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }
}