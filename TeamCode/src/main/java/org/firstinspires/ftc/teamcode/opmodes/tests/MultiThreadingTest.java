package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.util.Global;

@TeleOp(group = Global.OpModeGroup.EXPLORATORY)
public class MultiThreadingTest extends OpMode {

    DcMotor motor1;
    DcMotor motor2;

    NormalizedColorSensor colorSensor;

    volatile int color = 0;

    volatile boolean isActive = true;

    double prevTime = 0;

    Thread sensorThread = new Thread(() -> {
        while (isActive) {
            color = colorSensor.getNormalizedColors().toColor();
        }
    });

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(20);
    }

    @Override
    public void start() {
        sensorThread.start();
    }

    @Override
    public void loop() {
        double time = System.nanoTime() / 1e6;
        double dt = time - prevTime;
        prevTime = time;
        telemetry.addData("loopTime (ms)", dt);

        telemetry.addData("color", color);
        if (Math.abs(color) > 100) {
            motor2.setPower(0.8);
        } else {
            motor2.setPower(0.0);
        }
    }

    @Override
    public void stop() {
        isActive = false;
    }


}
