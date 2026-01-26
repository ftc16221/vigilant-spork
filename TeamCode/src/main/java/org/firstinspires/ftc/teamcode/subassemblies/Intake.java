package org.firstinspires.ftc.teamcode.subassemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Subassembly;

//@Config
public class Intake extends Subassembly {

    public static double SERVO_POWER = 0.8;

    private final CRServo intakeServo;

    public Intake(OpMode opMode) {
        super(opMode, "Intake");
        HardwareMap hardwareMap = opMode.hardwareMap;
        intakeServo = hardwareMap.crservo.get("intake");
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMode(Mode mode) {
        if (mode == Mode.IN) intakeServo.setPower(SERVO_POWER);
        else if (mode == Mode.OUT) intakeServo.setPower(-SERVO_POWER);
        else if (mode == Mode.OFF) intakeServo.setPower(0);
    }

    public void stop() {
        intakeServo.setPower(0);
    }

    public enum Mode {
        OUT, IN, OFF
    }
}
