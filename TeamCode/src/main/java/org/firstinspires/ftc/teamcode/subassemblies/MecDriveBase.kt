package org.firstinspires.ftc.teamcode.subassemblies

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.Subassembly
import org.firstinspires.ftc.teamcode.util.log
import org.firstinspires.ftc.teamcode.util.powerCurve
import kotlin.math.abs
import kotlin.math.max

class MecDriveBase(opMode: OpMode) : Subassembly(opMode, "Mecanum Drive Base") {

    val leftFront = hardwareMap.dcMotor.get("left_front") as DcMotorEx
    val rightFront = hardwareMap.dcMotor.get("right_front") as DcMotorEx
    val leftRear = hardwareMap.dcMotor.get("left_rear") as DcMotorEx
    val rightRear = hardwareMap.dcMotor.get("right_rear") as DcMotorEx

    private val motors = listOf(leftFront, rightFront, leftRear, rightRear)

    init {
        // direction = FORWARD by default
//        leftFront.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.REVERSE
//        leftRear.direction = DcMotorSimple.Direction.REVERSE
        rightRear.direction = DcMotorSimple.Direction.REVERSE

        opMode.log("DriveBase successfully initialized")
    }


    /**
     * Control the robot with a gamepad, usually called from a TeleOp
     *
     * @param gamepad the gamepad used to move the driveBase, usually gamepad1
     */
    fun control(gamepad: Gamepad) {

        zeroPowerBehavior = ZeroPowerBehavior.FLOAT

        val leftX = powerCurve(gamepad.left_stick_x.toDouble())
        val leftY = powerCurve(-gamepad.left_stick_y.toDouble())
        val rightX = gamepad.right_stick_x.toDouble()

        moveRobot(leftX, leftY, rightX)
    }

    /**
     * Moves the robot based on vectors between -1 and 1
     *
     * @param x strafe power
     * @param y forward/back power
     * @param yaw rotational power
     */
    fun moveRobot(x: Double, y: Double, yaw: Double) {
        // from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        val denominator = max(abs(y) + abs(x) + abs(yaw), 1.0)
        val leftFrontPower = (y + x + yaw) / denominator
        val rightFrontPower = (y - x - yaw) / denominator
        val leftRearPower = (y - x + yaw) / denominator
        val rightRearPower = (y + x - yaw) / denominator

        leftFront.power = leftFrontPower
        rightFront.power = rightFrontPower
        leftRear.power = leftRearPower
        rightRear.power = rightRearPower
    }

    fun telemetry() {
        telemetry.addLine()
        telemetry.addLine("surrender your soul to the squirrels")
    }

    var zeroPowerBehavior: ZeroPowerBehavior = ZeroPowerBehavior.UNKNOWN
        set(value) {
            for (motor in motors) motor.zeroPowerBehavior = value
            field = value
        }

    fun updateMaxRPM() {
        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }
    }
}