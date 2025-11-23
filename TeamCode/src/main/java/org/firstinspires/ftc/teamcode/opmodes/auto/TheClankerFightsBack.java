package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
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
@Configurable
public class TheClankerFightsBack extends OpMode {
    public static double X_P = -0.01;
    public static double X_I = 0;
    public static double X_D = 0;
    public static double A_P = 0.5;
    public static double A_I = 0;
    public static double A_D = 0;
    public static boolean ENABLE_TUNING_MODE = false;
    public static double DETECTION_TIME_OUT = 0.5;
    public static double SCANNING_POWER = 0.2;
    public static double MAX_POWER = 0.4;
    public static double AREA_SETPOINT = 0.90;
    public static int AVERAGING_SIZE = 10;
    public static int PIPELINE_INDEX = 9;

    Limelight3A limelight;
    MecDriveBase driveBase;

    RollingAverage txAvg = new RollingAverage(AVERAGING_SIZE);
    RollingAverage taAvg = new RollingAverage(AVERAGING_SIZE);


    PIDController xPidController = new PIDController(X_P, X_I, X_D);
    PIDController aPidController = new PIDController(A_P, A_I, A_D);


    ElapsedTime detectionTimer = new ElapsedTime();

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        driveBase = new MecDriveBase(this);
        xPidController.setSetPoint(0);
        aPidController.setSetPoint(AREA_SETPOINT);
        limelight.pipelineSwitch(PIPELINE_INDEX);
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

        LLResult result = limelight.getLatestResult();

        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
        for (LLResultTypes.DetectorResult detectorResult : detectorResults) {
            txAvg.addValue(detectorResult.getTargetXDegrees());
            taAvg.addValue(detectorResult.getTargetArea());
            detectionTimer.reset();
        }

        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
        for (LLResultTypes.ColorResult colorResult : colorResults) {
            txAvg.addValue(colorResult.getTargetXDegrees());
            taAvg.addValue(colorResult.getTargetArea());
            detectionTimer.reset();
        }

        double hPower = xPidController.calculate(txAvg.getAverage());
        double yPower = aPidController.calculate(taAvg.getAverage());

        if (detectionTimer.seconds() > DETECTION_TIME_OUT) {
            txAvg.reset();
            taAvg.reset();
            hPower = SCANNING_POWER;
            yPower = 0;
        }


        hPower = MathKt.clamp(hPower, -MAX_POWER, MAX_POWER);
        yPower = MathKt.clamp(yPower, -MAX_POWER, MAX_POWER);
        driveBase.moveRobot(0, yPower, hPower);

        telemetry.addData("num of detections", detectorResults.size());
        telemetry.addData("time since last detection", detectionTimer.seconds());
        telemetry.addData("txAvg", txAvg.getAverage());
        telemetry.addData("taAvg", taAvg.getAverage());

    }
}
