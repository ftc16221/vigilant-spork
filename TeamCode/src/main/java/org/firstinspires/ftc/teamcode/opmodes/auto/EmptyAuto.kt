package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Empty Autonomous")
class EmptyAuto: LinearOpMode() {

    override fun runOpMode() {
        waitForStart()

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addLine("Not much to see here")
                idle()
            }
        }
    }
}