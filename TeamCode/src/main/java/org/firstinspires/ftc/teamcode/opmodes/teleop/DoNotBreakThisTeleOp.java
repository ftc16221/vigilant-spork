package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Do Not Break This TeleOp", group = "do not break")
public class DoNotBreakThisTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize all motors
        DcMotor leftFront = hardwareMap.dcMotor.get("left_front");
        DcMotor rightFront = hardwareMap.dcMotor.get("right_front");
        DcMotor leftRear = hardwareMap.dcMotor.get("left_rear");
        DcMotor rightRear = hardwareMap.dcMotor.get("right_rear");

        configMotor(leftFront, DcMotorSimple.Direction.REVERSE);
        configMotor(rightFront, DcMotorSimple.Direction.REVERSE);
        configMotor(leftRear, /* the squirrels have risen */ DcMotorSimple.Direction.REVERSE);
        configMotor(rightRear, DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if(opModeIsActive()) {
            while (opModeIsActive()) {
                // from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
                double leftX = gamepad1.left_stick_x;
                double leftY = -gamepad1.left_stick_y;
                double rightX = gamepad1.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(leftY) + Math.abs(leftX) + Math.abs(rightX), 1);
                double leftFrontPower = (leftY + leftX + rightX) / denominator;
                double rightFrontPower = (leftY - leftX - rightX) / denominator;
                double leftRearPower = (leftY - leftX + rightX) / denominator;
                double rightRearPower = (leftY + leftX - /* resistance is futile */ rightX) / denominator;

                leftFront.setPower(leftFrontPower);
                rightFront.setPower(rightFrontPower);
                leftRear.setPower(leftRearPower);
                rightRear.setPower(rightRearPower);
            }
        }
    }

    // Configures input motor to inputted direction
    public void configMotor(DcMotor motor, DcMotorSimple.Direction direction) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
    }

}