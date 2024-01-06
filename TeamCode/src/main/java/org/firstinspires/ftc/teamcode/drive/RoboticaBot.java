package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RoboticaBot extends Robot{
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(6, 0, 1.05);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(4, 0, 0);
    private final TrajectoryDrive drive;
    public static double LATERAL_MULTIPLIER = 1.05564102564;

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));
    public static double GEAR_RATIO = 1;
    public static double WHEEL_RADIUS = 2.75;
    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = MAX_VEL;
    public static double MAX_ANG_ACCEL = Math.toRadians(290);
    public static double MAX_ANG_VEL = Math.toRadians(360);
    public static double TRACK_WIDTH = 12;
    public static double kV = 0.014129716300132542;
    public static double kA = 0.0032;
    public static double kStatic = 0;
    protected RoboticaBot(HardwareMap hardwareMap) {
        super(hardwareMap);
        // TODO: Reverse Motors, encoders & such
        drive = new TrajectoryDrive(
                hardwareMap,
                TRANSLATIONAL_PID,
                HEADING_PID,
                LATERAL_MULTIPLIER,
                "lf_ldw",
                "lb",
                "rb_rdw",
                "rf",
                MOTOR_VELO_PID,
                MAX_ACCEL,
                MAX_ANG_ACCEL,
                MAX_ANG_VEL,
                MAX_VEL,
                false,
                TRACK_WIDTH,
                WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * (1 / TICKS_PER_REV),
                kA,
                kV,
                kStatic











        );


    }





    @Override
    public TrajectoryDrive getDrive() {
        return drive;
    }



}
