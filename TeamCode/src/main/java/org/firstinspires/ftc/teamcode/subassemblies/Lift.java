package org.firstinspires.ftc.teamcode.subassemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Subassembly;

@Config
public class Lift extends Subassembly {

    DcMotor liftMotor;

    public Lift(OpMode opMode) {
        super(opMode, "Lift");

        liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double power) {
        liftMotor.setPower(power);
    }
}
