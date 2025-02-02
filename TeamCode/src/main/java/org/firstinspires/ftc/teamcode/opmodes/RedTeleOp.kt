package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subassemblies.AltClaw
import org.firstinspires.ftc.teamcode.subassemblies.LinearSlide
import org.firstinspires.ftc.teamcode.subassemblies.MecDriveBase
import org.firstinspires.ftc.teamcode.subassemblies.Underglow
import org.firstinspires.ftc.teamcode.util.log

@TeleOp(name = "Red TeleOp", group = "main")
class RedTeleOp: LightlessTeleOp() {

    override fun runOpMode() {
        val underglow = Underglow(this)
        underglow.setAlliance(Underglow.Alliance.RED)
        super.runOpMode()
    }
}