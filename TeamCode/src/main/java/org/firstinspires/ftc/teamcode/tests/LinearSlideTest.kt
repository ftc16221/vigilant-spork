package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(name = "Linear Slide Test")
class LinearSlideTest: LinearOpMode() {

    override fun runOpMode() {
        val slide = hardwareMap.dcMotor.get("linear_slide")
        val pinion = hardwareMap.crservo.get("carter's_opinion")

        slide.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        slide.mode = DcMotor.RunMode.RUN_USING_ENCODER

        waitForStart()

        if(opModeIsActive()) {
            while(opModeIsActive()) {
                slide.power = gamepad1.left_stick_y.toDouble()
                pinion.power = gamepad1.left_stick_x.toDouble()
            }
        }
    }
}