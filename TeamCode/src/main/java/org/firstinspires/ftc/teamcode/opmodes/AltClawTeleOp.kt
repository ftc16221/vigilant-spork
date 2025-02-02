package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subassemblies.AltClaw
import org.firstinspires.ftc.teamcode.subassemblies.LinearSlide
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase
import org.firstinspires.ftc.teamcode.util.log

@TeleOp(name = "Alt Claw TeleOp", group = "main")
class AltClawTeleOp: LinearOpMode() {

    override fun runOpMode() {
        // init, no movement allowed
        telemetry.isAutoClear = false

        val driveBase = MecDriveBase(this)
        val claw = AltClaw(this)
        val linearSlide = LinearSlide(this)
        // add other subassemblies here

        val loopTime = ElapsedTime()
//        val subassemblyList = listOf(driveBase)

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
                claw.control(gamepad2)
                linearSlide.control(gamepad2)
                // control other subassemblies here

                telemetry.addData("Loop Time", loopTime.time())
                telemetry.update()
            }
        }
    }
}