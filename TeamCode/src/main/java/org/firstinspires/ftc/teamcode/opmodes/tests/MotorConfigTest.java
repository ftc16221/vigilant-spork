package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.util.Global;

//@Config
@TeleOp(group = Global.OpModeGroup.TEST)
public class MotorConfigTest extends LinearOpMode {

    public static double MOTOR_POWER = 0.2;
    public static long MOTOR_RUN_TIME = 2000; // milliseconds

    @Override
    public void runOpMode() {
        MecDriveBase driveBase = new MecDriveBase(this);

        DcMotor leftFront = driveBase.leftFront;
        DcMotor rightFront = driveBase.rightFront;
        DcMotor leftRear = driveBase.leftRear;
        DcMotor rightRear = driveBase.rightRear;

        DcMotor[] motors = new DcMotor[]{ leftFront, rightFront, leftRear, rightRear };

        driveBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Assuming the motors are configured correctly then the following order of motors");
        telemetry.addLine("should move: leftFront, rightFront, leftRear, rightRear");
        telemetry.addLine("Ports:");
        telemetry.addData("left front ", leftFront.getPortNumber());
        telemetry.addData("right front", rightFront.getPortNumber());
        telemetry.addData("left rear  ", leftRear.getPortNumber());
        telemetry.addData("right rear ", rightRear.getPortNumber());
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                for (DcMotor motor : motors) {
                    motor.setPower(MOTOR_POWER);
                    sleep(MOTOR_RUN_TIME);
                    motor.setPower(0);
                }
            }
        }
    }
}