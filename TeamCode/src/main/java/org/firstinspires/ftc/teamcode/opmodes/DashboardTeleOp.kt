package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Vision
import org.firstinspires.ftc.teamcode.util.DashOpMode

@Disabled
@TeleOp(name = "Dash Camera")
class DashboardTeleOp : LinearOpMode(), DashOpMode {

    override fun runOpMode() {
        val driveBase = MecDriveBase(this)
        val vision = Vision(this)

        vision.visionPortal

        waitForStart()

        FtcDashboard.getInstance().startCameraStream(vision.dash, 0.0)

        while (opModeIsActive()) {
            sleep(100)
            driveBase.control(gamepad1)
        }
    }
}