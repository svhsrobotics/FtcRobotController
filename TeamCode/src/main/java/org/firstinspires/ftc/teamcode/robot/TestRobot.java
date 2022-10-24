package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.util.Timeout;

public class TestRobot extends Robot{

    static final double REV_COUNTS = 28;
    static final double GEAR_REDUCTION = 40;
    static final double WHEEL_DIAMETER = 3;

    private final HardwareMap hardwareMap;

    public TestRobot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void initHardware() {
        initDrives();
        initIMU();
    }

    public void initIMU() {
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

    public void initDrives() {
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
