package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.Subassembly
import kotlin.math.PI
import kotlin.math.roundToInt

class LinearSlide(opMode: OpMode): Subassembly(opMode, "Linear Slide") {

    val linearSlide = hardwareMap.dcMotor.get("linear_slide") as DcMotorEx
    val pinion = hardwareMap.servo.get("carter's_opinion")

    private var lastPosition = 0.0

    init {
        linearSlide.direction = DcMotorSimple.Direction.REVERSE
        linearSlide.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        linearSlide.mode = DcMotor.RunMode.RUN_USING_ENCODER

//        pinion.direction = Servo.Direction.REVERSE
    }

    /**
     * Uses the left stick of the provided gamepad
     */
    fun control(gamepad: Gamepad) {
        pinion.position += gamepad.left_stick_x * servoCoefficient

        if (gamepad.left_stick_y.toDouble() in -0.05..0.05) {
            yPosition = lastPosition
            linearSlide.mode = DcMotor.RunMode.RUN_TO_POSITION
        } else {
            linearSlide.mode = DcMotor.RunMode.RUN_USING_ENCODER
            linearSlide.power = - gamepad.left_stick_y.toDouble()
            lastPosition = yPosition
        }
    }

    /**
     * Sets the position of the claw assembly
     * the origin of x is the center of the robot
     * the origin of y is the bottom of the robot
     *
     * @param x in mm
     * @param y in mm
     */
    fun setPosition(x: Double, y: Double) {
        xPosition = x
        yPosition = y
    }

    var xPosition = 0.0
        set(value) {
            pinion.position = value / (servoGearDiameter * PI) * (360/300)
            field = value
        }
        get() = pinion.position * (300/360) * (servoGearDiameter * PI)

    var yPosition
        set(value) {
            linearSlide.targetPosition = (value / (motorGearDiameter * PI) * motorEncoderRes).roundToInt()
        }
        get() = linearSlide.currentPosition * (motorGearDiameter * PI) / motorEncoderRes

    companion object {
        @JvmField var motorEncoderRes = 1425.1
        @JvmField var motorGearDiameter = 39.0 // mm

        @JvmField var servoCoefficient = 0.001 // this value should be the highest possible without the pinion overshooting it's controls
        @JvmField var servoGearDiameter = 18.0 // mm
    }
}