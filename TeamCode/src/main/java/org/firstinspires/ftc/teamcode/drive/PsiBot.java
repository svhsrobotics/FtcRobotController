package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;

@Config
public class PsiBot extends Robot {
    public static double LEFTFORTYFIVE = -45;
    public static double LEFTSEVENTY = 70;
    public static double RIGHTFORTYFIVE = 45;
    public static double RIGHTSEVENTY = -70;
    private final AprilTagCamera[] cameras;
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);
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
    public static double WHEEL_RADIUS = 1.8898;
    public static double MAX_VEL = ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.85;
    public static double MAX_ACCEL = MAX_VEL;
    public static double MAX_ANG_ACCEL = Math.toRadians(80.66924999999999);
    public static double MAX_ANG_VEL = Math.toRadians(127.260357);
    public static double TRACK_WIDTH = 15.96;
    public static double kV = 0.014129716300132542;
    public static double kA = 0.0032;
    public static double kStatic = 0;

    private final Servo purpleServo;

    public PsiBot(HardwareMap hardwareMap) {
        super(hardwareMap);
        cameras = new AprilTagCamera[3];
        cameras[0] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Left"), 8, Math.toRadians(LEFTSEVENTY), Math.toRadians(LEFTFORTYFIVE));
        cameras[1] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Center"), 7, Math.toRadians(90), Math.toRadians(0));
        cameras[2] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Right"), 8, Math.toRadians(RIGHTSEVENTY), Math.toRadians(RIGHTFORTYFIVE));

        purpleServo = hardwareMap.get(Servo.class, "purple");

        // TODO: Reverse Motors, encoders & such
//        drive = new TrajectoryDrive(
//                hardwareMap,
//                TRANSLATIONAL_PID,
//                HEADING_PID,
//                LATERAL_MULTIPLIER,
//                "lf_ldw",
//                "lb",
//                "rb_rdw",
//                "rf",
//                MOTOR_VELO_PID,
//                MAX_ACCEL,
//                MAX_ANG_ACCEL,
//                MAX_ANG_VEL,
//                MAX_VEL,
//                false,
//                TRACK_WIDTH,
//                WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * (1 / TICKS_PER_REV),
//                kA,
//                kV,
//                kStatic
//        );
        drive = null;
    }

    @Override
    public AprilTagCamera[] getCameras() {
        return cameras;
    }

    @Override
    public void dropPurplePixel(boolean state) {
        if (state) {
            purpleServo.setPosition(0.3);
        } else {
            purpleServo.setPosition(0.5);
        }
    }

    @Override
    public TrajectoryDrive getDrive() {
        return drive;
    }
}
