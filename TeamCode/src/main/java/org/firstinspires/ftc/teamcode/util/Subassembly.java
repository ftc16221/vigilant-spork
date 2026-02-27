package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public abstract class Subassembly {

    protected final OpMode opMode;
    protected final String name;

    protected final Telemetry telemetry;
    protected final FtcDashboard dashboard;
    protected final HardwareMap hardwareMap;
    protected final double runtime;

    public Subassembly(OpMode opMode, String name) {
        this.opMode = opMode;
        this.name = name;

        this.telemetry = opMode.telemetry;
        this.dashboard = FtcDashboard.getInstance();
        this.hardwareMap = opMode.hardwareMap;
        this.runtime = opMode.getRuntime();
    }

    protected void sendData(String key, Object value) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put(key, value);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

//    public List<String> findIssues() {
//        return null;
//    }
}
