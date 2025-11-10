package org.firstinspires.ftc.teamcode.subassemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Subassembly;

@Config
public class Intake extends Subassembly {

    public static double POWER = 0.8;

    private final CRServo intakeServo;

    public Intake(OpMode opMode) {
        super(opMode, "Intake");
        HardwareMap hardwareMap = opMode.hardwareMap;
        intakeServo = hardwareMap.crservo.get("intake");
    }

    public void run(Direction direction) {
        if (direction == Direction.IN) intakeServo.setPower(POWER);
        else if (direction == Direction.OUT) intakeServo.setPower(-POWER);
    }

    public void stop() {
        intakeServo.setPower(0);
    }

    public enum Direction {
        OUT, IN
    }
}
