package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.autonomous.localizers.DriveEncoders;
import org.firstinspires.ftc.teamcode.util.Global;
import org.firstinspires.ftc.teamcode.util.Pose;

@Disabled
@TeleOp(group = Global.OpModeGroup.TUNER)
public class DeadReckoningTuner extends OpMode {

    MecDriveBase driveBase;
    DriveEncoders driveEncoders;

    public void init() {
        driveBase = new MecDriveBase(this);
        driveEncoders = new DriveEncoders(this, driveBase, new Pose(0, 0, 0));

        telemetry.addLine(
                "This OpMode is built to gather scalars for the Dead Reckoning localizer.\n" +
                        "To find the scalar, divide actual distance moved by reported distance.\n" +
                        "For example, if the robot is moved 100 cm, but reports 90 cm, then the scalar is 90/100 or 0.9.\n" +
                        "Alternatively, you can moved the robot until it reports 100 cm, then report the actual value.\n" +
                        "In this case, assuming the robot reported 111.1 cm, we would find the scalar to be 100/111.1 (~0.9)." +
                        "Do this for each direction - forward, lateral, rotational - then record the scalars to add permanently to DeadReckoning.java"
        );
    }

    @Override
    public void loop() {
        driveEncoders.update();
        telemetry.addData("recorded position", driveEncoders.getPose());
    }
}
