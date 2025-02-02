package org.firstinspires.ftc.teamcode.subassemblies

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.Subassembly

@Config
class AltClaw(opMode: OpMode) : Subassembly(opMode, "Alt Claw") {

    val clawServo = hardwareMap.servo.get("alt_claw")
    val rotateServo = hardwareMap.servo.get("alt_rotate")

    init {
        clawServo.direction = Servo.Direction.REVERSE
//        rotateServo.direction = Servo.Direction.REVERSE

        clawServo.scaleRange(clawScaleRange.first, clawScaleRange.second)
    }

    fun control(gamepad: Gamepad) {
        if (gamepad.y) open()
        if (gamepad.a) close()

        rotateServo.position += gamepad.right_stick_y * rotateServoCoefficient
    }

    fun open() { clawServo.position = 1.0 }
    fun close() { clawServo.position = 0.0 }

    companion object {
        @JvmField var rotateServoCoefficient = -0.003 // this value should be the highest possible without the pinion overshooting it's controls
        @JvmField var clawScaleRange = Pair(0.5, 0.725)
    }
}