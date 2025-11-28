package org.firstinspires.ftc.teamcode.navigation;

import static org.firstinspires.ftc.teamcode.util.Conversions.mmPerInch;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Date;
import java.util.List;

public class PlayingField {
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here

    public static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField  = 36 * mmPerInch;
    public static final float blockField = 24 * mmPerInch;

    public static Date date = new Date();

    // TODO: Look up dimensions and placement for all game field elements.

    // Known locations for field elements
    //
    public static final Position startingRingStack = new Position(DistanceUnit.INCH, quadField, blockField, 0, date.getTime());

    /** Square Target Zones **/
    /* If you are standing in the Red Alliance Station looking towards the center of the field,
     *     - The X axis runs from your left to the right. (positive from the center to the right)
     *     - The Y axis runs from the Red Alliance Station towards the other side of the field
     *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
     *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
     *
     */
    // Want four sets of corner coordinates to describe a square.
    public static final OpenGLMatrix[] targetSquareTranslations = {
            OpenGLMatrix.translation(blockField/2,-blockField/2, 0),
            OpenGLMatrix.translation(-blockField/2, -blockField/2, 0),
            OpenGLMatrix.translation(-blockField/2, blockField/2, 0),
            OpenGLMatrix.translation(blockField/2, blockField/2,0)
    };
    // Center of each target square; apply the targetSquareTranslations to the center point to get the full square.
    public static final Position redTargetZoneA = new Position(DistanceUnit.INCH, (blockField * (float)0.5), -(blockField * (float)2.5), 0, date.getTime());
    public static final Position redTargetZoneB = new Position(DistanceUnit.INCH, (blockField * (float)1.5), -(blockField * (float)1.5), 0, date.getTime());
    public static final Position redTargetZoneC = new Position(DistanceUnit.INCH, (blockField * (float)2.5), -(blockField * (float)2.5), 0, date.getTime());

    // we probably won't need these, but we can find them by reflecting the red targets across the X axis by negating their Y  values
    public static final Position blueTargetZoneA = new Position(DistanceUnit.INCH, redTargetZoneA.x, -redTargetZoneA.y, redTargetZoneA.z, date.getTime());
    public static final Position blueTargetZoneB = new Position(DistanceUnit.INCH, redTargetZoneB.x, -redTargetZoneB.y, redTargetZoneB.z, date.getTime());
    public static final Position blueTargetZoneC = new Position(DistanceUnit.INCH, redTargetZoneC.x, -redTargetZoneC.y, redTargetZoneC.z, date.getTime());
    /**/

    // These do not represent the center point; these are all lines.
    /** Starting and Launch lines **/
    // Want two sets of endpoint coordinates to describe a line segment.
    public static final OpenGLMatrix[] startingLineTranslations = {
            OpenGLMatrix.translation(0, 0, 0),
            OpenGLMatrix.translation(0, -blockField, 0)
    };
    public static final Position redStartingLine1 = new Position(DistanceUnit.INCH, -halfField, -blockField, 0, date.getTime());
    public static final Position redStartingLine2 = new Position(DistanceUnit.INCH, -halfField, -blockField * 2, 0, date.getTime());
    public static final Position blueStartingLine1 = new Position(DistanceUnit.INCH, redStartingLine1.x, -redStartingLine1.y, redStartingLine1.z, date.getTime());
    public static final Position blueStartingLine2 = new Position(DistanceUnit.INCH, redStartingLine2.x, -redStartingLine2.y, redStartingLine2.z, date.getTime());

    // I don't think we really care that the launch line is 2" wide tape.
    // all we need is its location across the field, which is its Z value.
    public static final float launchLineXLocation = -(blockField/2);
    /**/

    /** Goal Targets **/
    //TODO: Get positions of the center of each goal/target, and the translations to define the perimeter of each goal/target.
    public static final OpenGLMatrix[] goalTranslations = {

    };
    public static final List<OpenGLMatrix> redAllianceGoal = null;
    public static final List<OpenGLMatrix> redAlliancePowerShot = null;
    public static final List<OpenGLMatrix> blueAllianceGoal = null;
    public static final List<OpenGLMatrix> blueAlliancePowerShot = null;
    /**/

    /** VuMark Targets **/
    /*
     * Before being transformed, each target image is conceptually located at the origin of the field's
     *  coordinate system (the center of the field), facing up.
     */
    //TODO: Get positions of VuMark Targets - are these needed here, or are they available from Vuforia elsewhere?
    /**/
}

