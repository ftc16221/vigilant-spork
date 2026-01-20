package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Global;

@Autonomous(preselectTeleOp = Global.PRESELECT_TELEOP)
public class EmptyAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // intentionally blank
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                idle();
            }
        }
    }
}
