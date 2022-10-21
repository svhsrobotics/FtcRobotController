package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.hardware.Drive;

import java.util.EnumMap;

public abstract class Robot {

    public BNO055IMU imu = null;

    public enum DrivePos {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    public EnumMap<DrivePos, Drive> Drives = new EnumMap<DrivePos, Drive>(DrivePos.class);

    public abstract void initHardware();

}
