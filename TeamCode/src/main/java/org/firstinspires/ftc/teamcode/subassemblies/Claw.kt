package org.firstinspires.ftc.teamcode.subassemblies

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.degreesToServoPosition

class Claw(opMode: OpMode): Subassembly(opMode, "Claw") {

    val wrist = hardwareMap.servo.get("wrist")
    val leftHand = hardwareMap.crservo.get("left_hand")
    val rightHand = hardwareMap.crservo.get("right_hand")
    var handPower: Double = 0.0
        set(value) {
            leftHand.power = value
            rightHand.power = value
            field = value
        }

    init {
        wrist.direction = Servo.Direction.REVERSE
        leftHand.direction = DcMotorSimple.Direction.REVERSE
//        rightHand.direction = DcMotorSimple.Direction.REVERSE

        wrist.scaleRange(0.1, 0.6)
    }

    fun control(gamepad: Gamepad) {
        when {
            gamepad.dpad_up -> outtake()
            gamepad.dpad_down -> intake()
            gamepad.b || gamepad.dpad_left || gamepad.dpad_right -> stop()
        }
        when {
            gamepad.y -> wrist.raise()
            gamepad.a -> wrist.lower()
        }
    }

    fun intake() { handPower = 1.0 }
    fun outtake() { handPower = -1.0 }
    fun stop() { handPower = 0.0 }

    fun Servo.raise() { this.position = 1.0 }
    fun Servo.lower() { this.position = 0.0 }
}