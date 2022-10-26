package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.robot.hardware.Grabber;
import org.firstinspires.ftc.teamcode.util.Logger;

public class PowerPlayBot extends Robot {

    static final double REV_COUNTS = 28;
    static final double GEAR_REDUCTION = 40;
    static final double WHEEL_DIAMETER = 3;

    public Grabber grabber;

    public PowerPlayBot(HardwareMap hardwareMap, Logger logger) {
        super(hardwareMap, logger);
    }

    public PowerPlayBot(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void initHardware() {
        super.initHardware();
        initGrabber();
    }

    protected void initGrabber() {
        grabber = new Grabber(
                hardwareMap.get(DcMotor.class, "slide"),
                hardwareMap.get(DcMotor.class, "pitch"),
                hardwareMap.get(Servo.class, "wrist"),
                hardwareMap.get(CRServo.class, "pinch")
        );
    }

    @Override
    protected void initDrives() {
        drives.put(DrivePos.FRONT_LEFT,  new Drive(this.hardwareMap.get(DcMotorEx.class, "left_front"),  REV_COUNTS, GEAR_REDUCTION, WHEEL_DIAMETER));
        drives.put(DrivePos.FRONT_RIGHT, new Drive(this.hardwareMap.get(DcMotorEx.class, "right_front"), REV_COUNTS, GEAR_REDUCTION, WHEEL_DIAMETER));
        drives.put(DrivePos.BACK_LEFT,   new Drive(this.hardwareMap.get(DcMotorEx.class, "left_back"),   REV_COUNTS, GEAR_REDUCTION, WHEEL_DIAMETER));
        drives.put(DrivePos.BACK_RIGHT,  new Drive(this.hardwareMap.get(DcMotorEx.class, "right_back"),  REV_COUNTS, GEAR_REDUCTION, WHEEL_DIAMETER));

        for (Drive drive : drives.values())
            drive.run();

        // Shouldn't produce a NullPointerException because we literally just put drives in it
        drives.get(DrivePos.FRONT_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        drives.get(DrivePos.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
