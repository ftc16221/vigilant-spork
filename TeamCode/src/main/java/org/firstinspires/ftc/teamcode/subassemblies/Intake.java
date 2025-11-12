package org.firstinspires.ftc.teamcode.subassemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Subassembly;

@Config
public class Intake extends Subassembly {

    private final Servo intakeServo;

    public Intake(OpMode opMode) {
        super(opMode, "Intake");
        HardwareMap hardwareMap = opMode.hardwareMap;
        intakeServo = hardwareMap.servo.get("intake");
    }

    public void run(Direction direction) {
        if (direction == Direction.IN) intakeServo.setPosition(1);
        else if (direction == Direction.OUT) intakeServo.setPosition(0);
    }

    public void stop() {
        intakeServo.setPosition(0.5);
    }

    public enum Direction {
        OUT, IN
    }
}
