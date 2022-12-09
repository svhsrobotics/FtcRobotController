package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;
import org.firstinspires.ftc.teamcode.util.Logger;

public class PowerPlayBotV2 extends Robot {

    static final double REV_COUNTS = 28;
    static final double GEAR_REDUCTION = 40;
    static final double WHEEL_DIAMETER = 3;

    public Arm arm;

    public Webcam camera;

    public PowerPlayBotV2(HardwareMap hardwareMap, Logger logger) {
        super(hardwareMap, logger);
    }

    public PowerPlayBotV2(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void initHardware() {
        super.initHardware();
        initCamera();
        initArm();
    }

    protected void initCamera() {
        camera = new Webcam("Webcam 1", hardwareMap);
    }

    protected void initArm() {
        arm = new Arm(
               new Arm.Lift(
                       hardwareMap.get(DcMotor.class, "lift_low"),
                       hardwareMap.get(DcMotor.class, "lift_mid"),
                       hardwareMap.get(DcMotor.class, "lift_high")
               ),
                new Arm.Reacher(
                        hardwareMap.get(DcMotor.class, "reach")
                ),
                new Arm.Pincher(
                        hardwareMap.get(Servo.class, "pinch")
                )
        );

    }

    @Override
    protected void initDrives() {
        drives.put(DrivePos.FRONT_LEFT,  new Drive(this.hardwareMap.get(DcMotorEx.class, "front_left"),  REV_COUNTS, GEAR_REDUCTION, WHEEL_DIAMETER));
        drives.put(DrivePos.FRONT_RIGHT, new Drive(this.hardwareMap.get(DcMotorEx.class, "front_right"), REV_COUNTS, GEAR_REDUCTION, WHEEL_DIAMETER));
        drives.put(DrivePos.BACK_LEFT,   new Drive(this.hardwareMap.get(DcMotorEx.class, "back_left"),   REV_COUNTS, GEAR_REDUCTION, WHEEL_DIAMETER));
        drives.put(DrivePos.BACK_RIGHT,  new Drive(this.hardwareMap.get(DcMotorEx.class, "back_right"),  REV_COUNTS, GEAR_REDUCTION, WHEEL_DIAMETER));

        for (Drive drive : drives.values())
            drive.run();

        // Shouldn't produce a NullPointerException because we literally just put drives in it
        drives.get(DrivePos.FRONT_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
        drives.get(DrivePos.BACK_LEFT).setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
