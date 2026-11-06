/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */


package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;


/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2026-2027 FIRST® Tech Challenge. It leverages a differential/Skid-Steer system for robot mobility,
 * one motor driving an intake roller, two servos which pull elements out of corners, and a high-speed
 * launcher motor.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */
@Config
@Autonomous(name = "BioBuzz StarterBot GoHome Auto", group = "StarterBot")
//@Disabled
public class BioBuzzStarterBotGoHomeAuto extends LinearOpMode {
    private FtcDashboard dash;

    private boolean leftTriggerWasPressed = false;
    private boolean isRunning = false;
    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. We use the min velocity
     * in the launch() function to only run the windmill servo when the motor is spinning fast
     * enough to make a successful throw.
     */
    public static int LAUNCHER_TARGET_VELOCITY = 1275; // Ticks / second
    public static int LAUNCHER_MIN_VELOCITY = 1225;

    /*
     * These two variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the arcadeDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Create a variable to set to the intake.
    double intakePower;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void runOpMode() {

        // INIT
        dash = FtcDashboard.getInstance();
        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        MecDriveBase driveBase = new MecDriveBase(this);

        leftFrontDrive = driveBase.leftFront;
        rightFrontDrive = driveBase.rightFront;
        leftBackDrive = driveBase.leftBack;
        rightBackDrive = driveBase.rightBack;
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        driveBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");


        waitForStart();

        if (opModeIsActive()) {


            //go left
            // at time = 1000, it went 67 cm
            launch();
            mecanumDrive(0, 0.5, 0);
            sleep(1540);
            mecanumDrive(-0.5, 0, 0);
            sleep(2240);
            mecanumDrive(0, 0.5, 0);
            sleep(600);
            mecanumDrive(0, 0, 0);
            /*launch();
            mecanumDrive(0, -0.5, 0);
            sleep(1940);
            mecanumDrive(0, 0, 0);*/




        }
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }

    ElapsedTime launchTimer = new ElapsedTime();
    boolean launching = false;

    void launch() {
        ElapsedTime timer = new ElapsedTime();

        // Start launcher
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

        // Run for 25 seconds
        while (opModeIsActive() && timer.seconds() < 10) {

            // Wait for launcher to reach minimum velocity
            if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                windmillServo.setPower(1);

                intakePower = 0.5;
                intake.setPower(intakePower);
                leftIntakeServo.setPower(intakePower);
                rightIntakeServo.setPower(intakePower);
            }
            telemetry.addData("Flywheel Speed (TPS)", launcher.getVelocity());
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Flywheel Speed (TPS)", launcher.getVelocity());
            dash.sendTelemetryPacket(packet);
            // Small pause so the loop doesn't run unnecessarily fast
            idle();
        }

        // 25 seconds reached — shut everything off
        launcher.setVelocity(0);
        windmillServo.setPower(0);

        intakePower = 0;
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }
}