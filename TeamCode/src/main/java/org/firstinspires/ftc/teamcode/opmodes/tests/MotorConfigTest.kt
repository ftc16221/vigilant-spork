package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase
import org.firstinspires.ftc.teamcode.util.DashOpMode
import org.firstinspires.ftc.teamcode.util.Global

@TeleOp(group = Global.OpModeGroup.TEST)
class MotorConfigTest : LinearOpMode(), DashOpMode {

    override fun runOpMode() {
        val driveBase = MecDriveBase(this)

        val leftFront = driveBase.leftFront
        val rightFront = driveBase.rightFront
        val leftRear = driveBase.leftRear
        val rightRear = driveBase.rightRear

        val motors = listOf(leftFront, rightFront, leftRear, rightRear)

        driveBase.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        waitForStart()

        if (opModeIsActive()) {
            telemetry.addLine("Assuming the motors are configured correctly then the following order of motors")
            telemetry.addLine("should move: leftFront, rightFront, leftRear, rightRear")
            telemetry.addLine("Ports:")
            telemetry.addData("left front ", leftFront.portNumber)
            telemetry.addData("right front", rightFront.portNumber)
            telemetry.addData("left rear  ", leftRear.portNumber)
            telemetry.addData("right rear ", rightRear.portNumber)
            telemetry.update()

            while (opModeIsActive()) {
                for (motor in motors) motor.runFor(2000)
                sleep(2000)
            }
        }
    }

    // Run a motor for a set amount of time
    private fun DcMotor.runFor(milliseconds: Long) {
        power = MOTOR_POWER
        sleep(milliseconds)
        power = 0.0
    }

    companion object {
        const val MOTOR_POWER = 0.2
    }
}