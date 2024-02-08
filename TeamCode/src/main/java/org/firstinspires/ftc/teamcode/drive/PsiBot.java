package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Encoder;
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
//    public static final double TICKS_PER_REV = 537.7;
//    public static final double MAX_RPM = 312;
    public static final double TICKS_PER_REV = 383.6;
    public static final double MAX_RPM = 435;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));
    public static double GEAR_RATIO = 1;
    public static double WHEEL_RADIUS = 1.8898;
//    public static double MAX_VEL = ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.85;
    public static double MAX_VEL = 73.17330064499293;

    public static double MAX_ACCEL = MAX_VEL;
//    public static double MAX_ANG_ACCEL = Math.toRadians(80.66924999999999);
//    public static double MAX_ANG_VEL = Math.toRadians(127.260357);
    public static double MAX_ANG_VEL = Math.toRadians(258.001310769);
    public static double MAX_ANG_ACCEL = MAX_ANG_VEL;

//    public static double TRACK_WIDTH = 15.96;
    public static double TRACK_WIDTH = 16.25;

//    public static double kV = 0.014129716300132542;
     public static double kV = 0.01161656;

//    public static double kA = 0.0032;
    public static double kA = 0;
    public static double kStatic = 0;
    public static double DW_TICKS_PER_REV = 2048;
    public static double DW_WHEEL_RADIUS = 0.944882; // in
    public static double DW_GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 8; // in; distance between the left and right wheels
    //public static double FORWARD_OFFSET = -6.5; // in; offset of the lateral wheel
    public static double FORWARD_OFFSET = -7.5;

    private Encoder leftEncoder, rightEncoder, frontEncoder;
    public static double X_MULTIPLIER = 88.6/90; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 88.1/90; // Multiplier in the Y direction

    public final Servo purpleServo;
    public final Servo planeServo;
    public final DcMotor armMotor;
    public final Servo mosaicServo;

    public PsiBot(HardwareMap hardwareMap) {
        super(hardwareMap);
        cameras = new AprilTagCamera[3];
        cameras[0] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Left"), 5.5, -6.5);
        cameras[1] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Center"), 0, 0);
        cameras[2] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Right"), 5.5, 6.5);

        purpleServo = hardwareMap.get(Servo.class, "purple");
        planeServo = hardwareMap.get(Servo.class, "plane");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        mosaicServo = hardwareMap.get(Servo.class, "mosaic");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         //TODO: Reverse Motors, encoders & such

        drive = new TrajectoryDrive(
                hardwareMap,
                TRANSLATIONAL_PID,
                HEADING_PID,
                LATERAL_MULTIPLIER,
                "front_left",
                "back_left",
                "back_right_back_pod",
                "front_right_right_pod",
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
                "left_pod",
                "front_right_right_pod",
                "back_right_back_pod", // TODO: FIX THESE
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

        ((TrackingWheelLocalizer)drive.getLocalizer()).leftEncoder.setDirection(Encoder.Direction.REVERSE);
        //((TrackingWheelLocalizer)drive.getLocalizer()).frontEncoder.setDirection(Encoder.Direction.REVERSE);
        //((TrackingWheelLocalizer)drive.getLocalizer()).rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    @Override
    public AprilTagCamera[] getCameras() {
        return cameras;
    }

    @Override
    public AprilTagCamera getPrimaryCamera() {
        return cameras[1];
    }

    @Override
    public void dropPurplePixel(boolean state) {
        if (state) {
            //purpleServo.setPosition(1);
            for (double i = 0; i < 0.7; i += 0.1) {
                purpleServo.setPosition(i);
                //GlobalOpMode.opMode.sleep(10);
            }
        } else {
            purpleServo.setPosition(0);
        }
    }

    @Override
    public void launchPlane() {
        // TODO: Implement
    }

    @Override
    public TrajectoryDrive getDrive() {
        return drive;
    }
}
