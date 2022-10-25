package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.util.Timeout;

import java.util.EnumMap;

public abstract class Robot {

    protected final HardwareMap hardwareMap;

    public BNO055IMU imu;

    public enum DrivePos {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    public EnumMap<DrivePos, Drive> drives = new EnumMap<>(DrivePos.class);

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void initHardware() {
        initIMU();
        initDrives();
    }

    protected void initIMU() {
        // Get the imu
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set the IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        // Initialize the imu with specified parameter
        imu.initialize(parameters);

        // Wait while the gyro is calibrating...
        while (!imu.isGyroCalibrated()) {
            Thread.yield();
        }

        Timeout t = new Timeout(5);
        while (!imu.isAccelerometerCalibrated() && !t.expired()) {
            Thread.yield();
        }
    }

    /**
     * This method must be overridden by the child class
     * get motors from the hardware map and initialize them
     * in the drives EnumMap
     */
    protected abstract void initDrives();


}
