package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.hardware.Drive;

public class TestRobot extends Robot{

    static final double REV_COUNTS = 28;
    static final double GEAR_REDUCTION = 40;
    static final double WHEEL_DIAMETER = 3;

    public TestRobot(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    protected void initDrives() {
        drives.put(DrivePos.FRONT_LEFT,  new Drive(this.hardwareMap.get(DcMotorEx.class, "left_front"),  REV_COUNTS, GEAR_REDUCTION, WHEEL_DIAMETER));
        drives.put(DrivePos.FRONT_RIGHT, new Drive(this.hardwareMap.get(DcMotorEx.class, "right_front"), REV_COUNTS, GEAR_REDUCTION, WHEEL_DIAMETER));
        drives.put(DrivePos.BACK_LEFT,   new Drive(this.hardwareMap.get(DcMotorEx.class, "left_back"),   REV_COUNTS, GEAR_REDUCTION, WHEEL_DIAMETER));
        drives.put(DrivePos.BACK_RIGHT,  new Drive(this.hardwareMap.get(DcMotorEx.class, "right_back"),  REV_COUNTS, GEAR_REDUCTION, WHEEL_DIAMETER));

        for (Drive drive : drives.values())
            drive.run();

        drives.get(DrivePos.FRONT_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        drives.get(DrivePos.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
