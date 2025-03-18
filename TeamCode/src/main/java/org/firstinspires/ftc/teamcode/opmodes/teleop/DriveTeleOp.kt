package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase
import org.firstinspires.ftc.teamcode.util.log

@TeleOp(name = "Drive TeleOp", group = "!main")
class DriveTeleOp: LinearOpMode() {

    override fun runOpMode() {
        // init, no movement allowed
        telemetry.isAutoClear = false

        val driveBase = MecDriveBase(this)
        // add other subassemblies here

        val loopTime = ElapsedTime()
        val subassemblyList = listOf(driveBase)

        log("OpMode initialized")
        waitForStart()

        if (opModeIsActive()) {
            log("Starting OpMode loop")
            telemetry.isAutoClear = true
            telemetry.clear()
            while (opModeIsActive()) {
                loopTime.reset()
                telemetry.addData("G1 Left X", gamepad1.left_stick_x)
                telemetry.addData("G1 Left Y", gamepad1.left_stick_y)
                telemetry.addData("G1 Right X", gamepad1.right_stick_x)
                telemetry.addData("G1 Right XY", gamepad1.right_stick_y)

                // Subassembly control
                driveBase.control(gamepad1)
                // control other subassemblies here

                telemetry.addData("Loop Time", loopTime.time())
                telemetry.update()
            }
        }
    }
}