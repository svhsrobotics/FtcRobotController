package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;

@Config
public class TestBot extends Robot {
    public static double LEFTFORTYFIVE = -45;
    public static double LEFTSEVENTY = 70;
    public static double RIGHTFORTYFIVE = 45;
    public static double RIGHTSEVENTY = -70;
    private final AprilTagCamera[] cameras;
    private final AprilTagCamera primaryCamera;
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);
    private final TrajectoryDrive drive;
    public static double LATERAL_MULTIPLIER = 1.05564102564;

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
    public static final double TICKS_PER_REV = 1120;
    public static final double MAX_RPM = 150;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));
    public static double GEAR_RATIO = 1;
    public static double WHEEL_RADIUS = 1.4763; // in
    public static double MAX_VEL = 19.7112162466021;
    public static double MAX_ACCEL = MAX_VEL;
    public static double MAX_ANG_ACCEL = Math.toRadians(80.66924999999999);
    public static double MAX_ANG_VEL = Math.toRadians(127.260357);
    public static double TRACK_WIDTH = 13.98; // in
    public static double kV = 0.042122656124607556;
    public static double kA = 0.002;
    public static double kStatic = 0.01;

    public static double DW_TICKS_PER_REV = 2048;
    public static double DW_WHEEL_RADIUS = 0.944882; // in
    public static double DW_GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14.863; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -9; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 88.6/90; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 88.1/90; // Multiplier in the Y direction

    //private final Servo purpleServo;

    public TestBot(HardwareMap hardwareMap) {
        super(hardwareMap);
        if (hardwareMap.tryGet(WebcamName.class, "Left") != null && hardwareMap.tryGet(WebcamName.class, "Right") != null) {
            cameras = new AprilTagCamera[3];

            cameras[0] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Left"), 8, Math.toRadians(LEFTSEVENTY), Math.toRadians(LEFTFORTYFIVE));
            cameras[1] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Center"), 7, Math.toRadians(90), Math.toRadians(0));
            cameras[2] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Right"), 8, Math.toRadians(RIGHTSEVENTY), Math.toRadians(RIGHTFORTYFIVE));

            primaryCamera = cameras[1];
        } else {
            cameras = new AprilTagCamera[1];
            cameras[0] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Center"), 7, Math.toRadians(90), Math.toRadians(0));
            primaryCamera = cameras[0];
        }
        //purpleServo = hardwareMap.get(Servo.class, "purple");

        // TODO: Reverse Motors, encoders & such
        drive = new TrajectoryDrive(
                hardwareMap,
                TRANSLATIONAL_PID,
                HEADING_PID,
                LATERAL_MULTIPLIER,
                "left_front_left_dw",
                "left_back",
                "right_back_right_dw",
                "right_front",
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
                "no_motor_perp_dw",
                X_MULTIPLIER,
                Y_MULTIPLIER,
                FORWARD_OFFSET,
                LATERAL_DISTANCE,
                DW_GEAR_RATIO,
                DW_WHEEL_RADIUS,
                DW_TICKS_PER_REV
        );
        drive.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        ((TrackingWheelLocalizer)drive.getLocalizer()).rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }



    @Override
    public AprilTagCamera[] getCameras() {
        return cameras;
    }

    @Override
    public AprilTagCamera getPrimaryCamera() {
        return primaryCamera;
    }

    @Override
    public void dropPurplePixel(boolean state) {
//        if (state) {
//            purpleServo.setPosition(0.3);
//        } else {
//            purpleServo.setPosition(0.5);
//        }
    }

    @Override
    public void launchPlane() {
        // TODO
    }

    @Override
    public TrajectoryDrive getDrive() {
        return drive;
    }
}
