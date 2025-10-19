package org.firstinspires.ftc.teamcode.subassemblies;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Subassembly;

public class Intake extends Subassembly {

    public static double POWER = 0.8;

    private final DcMotor intakeMotor;


    public Intake(OpMode opMode) {
        super(opMode, "Intake");
        HardwareMap hardwareMap = opMode.hardwareMap;
        intakeMotor = hardwareMap.dcMotor.get("intake");
    }

    public void run(Direction direction) {
        if (direction == Direction.IN) intakeMotor.setPower(POWER);
        else if (direction == Direction.OUT) intakeMotor.setPower(-POWER);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    public enum Direction {
        OUT, IN
    }
}
