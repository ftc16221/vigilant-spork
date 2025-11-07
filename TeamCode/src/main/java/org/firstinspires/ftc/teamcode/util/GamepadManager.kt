package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Gamepad

class GamepadManager(val gamepad: Gamepad) {
    var wasPressed = mutableSetOf<String>()
    private fun getButton(name: String) = Gamepad::class.java.getDeclaredField(name).getBoolean(gamepad)
    fun on(button: String, closure: () -> Unit) {
        if(getButton(button)) {
            wasPressed.add(button)
            closure()
        }
    }
    fun off(buttons: Array<String>, closure: () -> Unit) {
        if(wasPressed.any { buttons.contains(it) } && !buttons.any { getButton(it) }) {
            wasPressed.removeAll(buttons.toSet())
            closure()
        }
    }
    fun once(button: String, closure: () -> Unit) {
        if(getButton(button)) {
            if(!wasPressed.contains(button)) {
                closure()
            }
            wasPressed.add(button)
        } else {
            wasPressed.remove(button)
        }
    }
}