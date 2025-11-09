package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.MathKt;
import org.firstinspires.ftc.teamcode.util.RollingAverage;

import java.util.List;

@Autonomous(group = Global.OpModeGroup.EXPLORATORY)
@Config
public class TheClankerFightsBack extends OpMode {
    public static double X_P = -0.01;
    public static double X_I = 0;
    public static double X_D = 0;
    public static double A_P = -0.01;
    public static double A_I = 0;
    public static double A_D = 0;
    public static boolean ENABLE_TUNING_MODE = false;
    public static double DETECTION_TIME_OUT = 0.5;
    public static double MAX_POWER = 0.4;
    public static double AREA_SETPOINT = 0.90;

    Limelight3A limelight;
    MecDriveBase driveBase;

    RollingAverage txAvg = new RollingAverage(50);
    RollingAverage taAvg = new RollingAverage(50);


    PIDController xPidController = new PIDController(X_P, X_I, X_D);
    PIDController aPidController = new PIDController(A_P, A_I, A_D);


    ElapsedTime detectionTimer = new ElapsedTime();

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        driveBase = new MecDriveBase(this);
        xPidController.setSetPoint(0);
        aPidController.setSetPoint(AREA_SETPOINT);
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        FtcDashboard.getInstance().startCameraStream(limelight, 40);
    }

    @Override
    public void loop() {

        if(ENABLE_TUNING_MODE) {
            xPidController.setPID(X_P, X_I, X_D);
            aPidController.setPID(A_P, A_I, A_D);
            aPidController.setSetPoint(AREA_SETPOINT);
        }

        List<LLResultTypes.DetectorResult> detections = limelight.getLatestResult().getDetectorResults();
        for (LLResultTypes.DetectorResult detection : detections) {
            txAvg.addValue(detection.getTargetXDegrees());
            taAvg.addValue(detection.getTargetArea());
            detectionTimer.reset();
        }

        if (detectionTimer.seconds() > DETECTION_TIME_OUT) {
            txAvg.reset();
        }

        double hPower = xPidController.calculate(txAvg.getAverage());
        hPower = MathKt.clamp(hPower, -MAX_POWER, MAX_POWER);
        double yPower = aPidController.calculate(taAvg.getAverage());
        driveBase.moveRobot(0, 0, hPower);

        telemetry.addData("num of detections", detections.size());
        telemetry.addData("time since last detection", detectionTimer.seconds());
        telemetry.addData("txAvg", txAvg.getAverage());
        telemetry.addData("taAvg", taAvg.getAverage());

    }
}
