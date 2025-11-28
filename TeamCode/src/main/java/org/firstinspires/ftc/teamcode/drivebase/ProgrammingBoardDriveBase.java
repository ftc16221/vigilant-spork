package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ProgrammingBoardDriveBase extends DriveBase {

    //@Device("left_front")
    public DcMotor motorLeftFront;

    public ProgrammingBoardDriveBase(HardwareMap hwMap) {
        super(hwMap);
    }

    @Override
    protected void initMotorConfigurations() {
        motorLeftFront  = hardwareMap.dcMotor.get("left_front");
        wheelMotors = new DcMotor[]{ motorLeftFront };

        motorConfigurations.put(TravelDirection.forward, new DcMotorSimple.Direction[]{ DcMotorSimple.Direction.FORWARD });
        motorConfigurations.put(TravelDirection.reverse, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE});

        motorConfigurations.put(TravelDirection.pivotLeft, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE});
        motorConfigurations.put(TravelDirection.pivotRight, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD});

        motorConfigurations.put(TravelDirection.strafeLeft, new DcMotorSimple.Direction[]{ DcMotorSimple.Direction.FORWARD });
        motorConfigurations.put(TravelDirection.strafeLeftBackward, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE});
        motorConfigurations.put(TravelDirection.strafeLeftForward, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD});

        motorConfigurations.put(TravelDirection.strafeRight, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE});
        motorConfigurations.put(TravelDirection.strafeRightBackward, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD});
        motorConfigurations.put(TravelDirection.strafeRightForward, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.REVERSE});

        // super.initMotorConfigurations();
    }
}
