package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subassemblies.Underglow;
import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(name = "Set Alliance Blue", group = Global.OpModeGroup.SETUP)
@Disabled
public class SetAlliance extends LinearOpMode {

    protected int brightness = Global.defaultUnderglowBrightness;
    private int opIterationCounter = 0;
    protected Underglow underglow;
    private Telemetry.Item tBrightness;
    private Telemetry.Item tEnabled;
    private Telemetry.Item tAlliance;
    private Telemetry.Item tIteration;

    public Global.Alliance getAlliance() {
        return Global.getAlliance();
    }
    public void setAlliance(Global.Alliance alliance) {
        Global.setAlliance(alliance);
    }

    /**
     * Please override me to set the alliance color.
     * use super() to include the standard telemetry.
     */
    public void setAllianceColor() {
        // setAlliance([YOUR COLOR HERE]);
        tAlliance.setValue(getAlliance());
    }

    public void initTelemetry() {
        telemetry.setAutoClear(false);
        telemetry.clear();
        tAlliance = telemetry.addData("Alliance set to", getAlliance());
        tBrightness = telemetry.addData("Brightness", brightness);
        telemetry.addLine();
        telemetry.addLine("Press B to re-initialize underglow\nPress X to disable underglow\nUse bumpers to control brightness");
        tIteration = telemetry.addData("Iteration", opIterationCounter);
    }
    public void initUnderglow() {
        // re-initialize
        underglow = null;
        underglow = new Underglow(this);

        underglow.setColorToAlliance();
        tBrightness.setValue(brightness);
    }

    public void useControls() {

        if (gamepad1.leftBumperWasReleased()) {
            if (brightness > 1) brightness--;
            underglow.setBrightness(brightness);
            tBrightness.setValue(brightness);
        }
        if (gamepad1.rightBumperWasReleased()) {
            if (brightness < 29) brightness++;
            underglow.setBrightness(brightness);
            tBrightness.setValue(brightness);
        }
        if (gamepad1.bWasReleased()) {
            // re-initialize.  See if that helps get things unstuck.
            initUnderglow();
            idle();
        }
        if (gamepad1.xWasReleased()) {
            underglow.off();
            idle();
        }
    }

    @Override
    public void runOpMode() {
        initTelemetry();
        setAllianceColor();
        initUnderglow();
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            while(opModeIsActive()) {
                tIteration.setValue(opIterationCounter++);
                useControls();
                telemetry.update();
                idle();
            }
        }
    }
}