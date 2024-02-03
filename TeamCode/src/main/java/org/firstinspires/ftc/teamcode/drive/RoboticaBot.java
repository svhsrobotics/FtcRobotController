package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;

public class RoboticaBot extends Robot {
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
    public static double X_MULTIPLIER = 1.009485424;
    public static double Y_MULTIPLIER = 1.017838563;
    public static double FORWARD_OFFSET = -4.25;
    public static double LATERAL_DISTANCE = 15.83397;
    public static double DW_GEAR_RATIO = 1;
    public static double DW_WHEEL_RADIUS = 0.944882;
    public static double DW_TICKS_PER_REV = 2048;

    public RoboticaBot(HardwareMap hardwareMap) {
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
                kStatic,
                "left_front_left_dw",
                "right_back_right_dw",
                "no_motor_perp_dw", // TODO: FIX THESE
                X_MULTIPLIER,
                Y_MULTIPLIER,
                FORWARD_OFFSET,
                LATERAL_DISTANCE,
                DW_GEAR_RATIO,
                DW_WHEEL_RADIUS,
                DW_TICKS_PER_REV
        );
    }

    @Override
    public AprilTagCamera[] getCameras() {
        return new AprilTagCamera[0];
    }

    @Override
    public AprilTagCamera getPrimaryCamera() {
        return null;
    }

    @Override
    public void dropPurplePixel(boolean state) {
        // TODO
    }

    @Override
    public void launchPlane() {
        // TODO
    }

    @Override
    public TrajectoryDrive getDrive() {
        return null;
    }
}
