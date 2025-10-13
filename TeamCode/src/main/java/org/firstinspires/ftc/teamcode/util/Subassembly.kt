package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class Subassembly(protected val opMode: OpMode, protected val name: String? = null) {

    protected val telemetry: MultipleTelemetry = MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().telemetry)
    protected val hardwareMap: HardwareMap = opMode.hardwareMap
    protected val runtime = opMode.runtime
}