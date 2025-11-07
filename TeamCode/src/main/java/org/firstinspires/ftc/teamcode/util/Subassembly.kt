package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap

abstract class Subassembly(protected val opMode: OpMode, protected val name: String? = null) {

    protected val telemetry: MultipleTelemetry = MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().telemetry)
    protected val hardwareMap: HardwareMap = opMode.hardwareMap
    protected val runtime = opMode.runtime

    protected fun sendData(key: String, value: Any) {
        val packet = TelemetryPacket()
        packet.put(key, value)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }
}