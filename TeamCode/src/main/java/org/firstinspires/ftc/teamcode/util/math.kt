@file:Suppress("unused")
package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.*

fun clamp(value: Double, lowerBound: Double, upperBound: Double)
    = max(lowerBound, min(upperBound, value))

fun equalsTolerance(a: Double, b: Double, tolerance: Double)
    = abs(b - a) < tolerance

fun encoderPositionToDegrees(encoderPosition: Int, encoderResolution: Double)
    = 360 * encoderPosition / encoderResolution

fun degreesToEncoderPosition(degrees: Double, encoderResolution: Double)
    = (degrees / 360.0 * encoderResolution).toInt()

/** Assumes use of a 300 degree non-continuous servo */
fun degreesToServoPosition(degrees: Double, scaleRange: Pair<Double, Double>): Double {
    val scale = abs(scaleRange.first - scaleRange.second)
    return degrees / (scale * 300) - (0.5 * scale)
}

/** **radians** to degrees */
fun Double.toDegrees() = this * 180 / PI
/** **degrees** to radians */
fun Double.toRadians() = this * PI / 180

fun normalize(angle: Double) =
    if (Global.ANGLE_UNIT == AngleUnit.DEGREES) AngleUnit.normalizeDegrees(angle)
    else AngleUnit.normalizeRadians(angle)

/** **centimeters** to inches */
fun Double.toInches() = this * 2.54
/** **inches** to centimeters */
fun Double.toCentimeters() = this / 2.54

fun powerCurve(value: Double) = value.pow(2).withSign(value)