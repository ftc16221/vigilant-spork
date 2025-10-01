package org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Pose;

public class LimelightCam extends Localizer {

    private final Limelight3A limelight3A;

    public LimelightCam(LinearOpMode opMode) {
        super(opMode, "Limelight 3A");
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
    }

    @Override public void update() {
        LLResult result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                pose = new Pose(result.getBotpose());
            }
        } else {
            pose = null;
        }
    }
}
