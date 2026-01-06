package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.RobotLog

@Deprecated("use Watchdog.log instead")
fun OpMode.log(message: String) {
    val tag = this::class.simpleName ?: "unknown opmode"

    telemetry.addLine("> $message")
    RobotLog.i("($tag) $message")
    telemetry.update()
}