package org.firstinspires.ftc.teamcode.subassemblies

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.Subassembly
import kotlin.math.PI
import kotlin.math.roundToInt

@Config
class LinearSlide(opMode: OpMode): Subassembly(opMode, "Linear Slide") {

    @JvmField var HIGH_BASKET_POS = 38.4
    @JvmField var HIGH_RUNG_POS = 30.0
    @JvmField var PICKUP_POS = 3.0
    @JvmField var ASCEND_POS = 20

    val linearSlide = hardwareMap.dcMotor.get("linear_slide") as DcMotorEx
    val pinion = hardwareMap.crservo.get("carter's_opinion")

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
        pinion.power = gamepad.left_stick_x.toDouble()

        if (gamepad.left_stick_y.toDouble() in -0.05..0.05) {
            yPosition = lastPosition
            linearSlide.mode = DcMotor.RunMode.RUN_TO_POSITION
        } else {
            linearSlide.mode = DcMotor.RunMode.RUN_USING_ENCODER
            linearSlide.power = - gamepad.left_stick_y.toDouble()
            lastPosition = yPosition
        }
    }

    var yPosition
        set(value) {
            linearSlide.targetPosition = (value / (motorGearDiameter * PI) * motorEncoderRes).roundToInt()
        }
        get() = linearSlide.currentPosition * (motorGearDiameter * PI) / motorEncoderRes

    /**
     * @param position desired position in inches
     * @param power power the slides will be ran at
     */
    fun moveSlide(position: Double, power: Double) {
        linearSlide.targetPosition = (position / (motorGearDiameter * PI) * motorEncoderRes).toInt()
        linearSlide.mode = DcMotor.RunMode.RUN_TO_POSITION
        linearSlide.power = power
    }

    companion object {
        @JvmField var motorEncoderRes = 751.8 // PPR
        @JvmField var motorGearDiameter = 1.54 // in

        @JvmField var servoCoefficient = 0.001 // this value should be the highest possible without the pinion overshooting it's controls
        @JvmField var servoGearDiameter = 18.0 // mm
    }
}