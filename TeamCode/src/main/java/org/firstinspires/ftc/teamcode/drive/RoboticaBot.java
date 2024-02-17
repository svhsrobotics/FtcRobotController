package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.AxonServo;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;

@Config
public class RoboticaBot extends Robot {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(6, 0, 1.05);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(4, 0, 0);
    private final TrajectoryDrive drive;
    public static double LATERAL_MULTIPLIER = 1.05564102564;

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312;

    // UNUSED B/C NOT USING ENCODERS
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));
    public static double GEAR_RATIO = 1;
    public static double WHEEL_RADIUS = 2.75591;
    /*
     * Note from LearnRoadRunner.com:
     * The velocity and acceleration constraints were calculated based on the following equation:
     * ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.85
     * Resulting in 76.53624803199908 in/s.
     * This is only 85% of the theoretical maximum velocity of the bot, following the recommendation above.
     * This is capped at 85% because there are a number of variables that will prevent your bot from actually
     * reaching this maximum velocity: voltage dropping over the game, bot weight, general mechanical inefficiencies, etc.
     * However, you can push this higher yourself if you'd like. Perhaps raise it to 90-95% of the theoretically
     * max velocity. The theoretically maximum velocity is 90.04264474352833 in/s.
     * Just make sure that your bot can actually reach this maximum velocity. Path following will be detrimentally
     * affected if it is aiming for a velocity not actually possible.
     *
     * The maximum acceleration is somewhat arbitrary and it is recommended that you tweak this yourself based on
     * actual testing. Just set it at a reasonable value and keep increasing until your path following starts
     * to degrade. As of now, it simply mirrors the velocity, resulting in 76.53624803199908 in/s/s
     *
     * Maximum Angular Velocity is calculated as: maximum velocity / trackWidth * (180 / Math.PI) but capped at 360°/s.
     * You are free to raise this on your own if you would like. It is best determined through experimentation.
     */
    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = MAX_VEL;
    public static double MAX_ANG_ACCEL = Math.toRadians(360);
    public static double MAX_ANG_VEL = Math.toRadians(360);
    public static double TRACK_WIDTH = 12;

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }
    public static double kV = 0.0135;
    public static double kA = 0.004;
    public static double kStatic = 0.01;
    public static double X_MULTIPLIER = 1;
    public static double Y_MULTIPLIER = 1;
    public static double FORWARD_OFFSET = -3.0;
    public static double LATERAL_DISTANCE = 16.083;
    public static double DW_GEAR_RATIO = 1;
    public static double DW_WHEEL_RADIUS = 0.944882;
    public static double DW_TICKS_PER_REV = 2000;

    private final AprilTagCamera[] cameras;

//    private final Servo purpleServo;
//    public final DcMotorEx armMotor;
//    public final Servo wristServo;
//    public final AxonServo intakeServo;
//    public final Servo droneServo;
//    public final DcMotorEx hangMotor;
//    public final DcMotor intakeMotor;

    public final DcMotorEx shoulderMotor;
    public final DcMotorEx hangMotor;
    public final AxonServo elbowServo;
    //public final AxonServo wristTwistServo;
    public final Servo wristTwistServo;
    public final Servo wristLiftServo;
    public final AxonServo pinchServo;
    public final Servo purpleServo;
    public final Servo planeAngleServo;
    public final CRServo planeReleaseServo;
    public final TouchSensor magneticLimitSwitch;

    public RoboticaBot(HardwareMap hardwareMap) {
        super(hardwareMap);
        cameras = new AprilTagCamera[3];
        cameras[0] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Left"), 5.5, -5.5);
        cameras[1] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Center"), 0, 0);
        cameras[2] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Right"), 5.5, 5.5);


        // plane_release crservo
        // plane_angle servo

        // shoulder motor
        // hang motor
        // elbow axon servo
        // wrist_twist axon servo
        // wrist_lift servo
        // pinch axon servo
        // purple servo

        // axon_2 analog
        // axon_3 analog

        planeAngleServo = hardwareMap.get(Servo.class, "plane_angle");
        planeReleaseServo = hardwareMap.get(CRServo.class, "plane_release");

        shoulderMotor = hardwareMap.get(DcMotorEx.class, "shoulder");
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor = hardwareMap.get(DcMotorEx.class, "hang");
        elbowServo = new AxonServo("elbow", "axon_2", hardwareMap);
        //wristTwistServo = new AxonServo("wrist_twist", "axon_3", hardwareMap);
        wristTwistServo = hardwareMap.get(Servo.class, "wrist_twist");
        wristLiftServo = hardwareMap.get(Servo.class, "wrist_lift");
        //pinchServo = hardwareMap.get(AxonServo.class, "pinch");
        pinchServo = new AxonServo("pinch", "axon_pinch", hardwareMap); // TODO SWITCH TO NORMAL SERVO
        //pinchServo = hardwareMap.get(Servo.class, "pinch");
        //pinchServo.setPower(0.1);
        purpleServo = hardwareMap.get(Servo.class, "purple");

        magneticLimitSwitch = hardwareMap.get(TouchSensor.class, "magnetic_limit_switch");



//        purpleServo = hardwareMap.get(Servo.class, "purple");
//        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        wristServo = hardwareMap.get(Servo.class, "cup_pivot");
//        intakeServo = new AxonServo("intake_servo", "intake_analog_2", hardwareMap);
//        droneServo = hardwareMap.get(Servo.class, "drone");
//        hangMotor = hardwareMap.get(DcMotorEx.class, "hang");
//        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        // TODO: Reverse Motors, encoders & such

        /*
        arm
        intake
        hang
        ppp
        cup_pivot
        intake_servo
        intake_analog_2 / intake_analog_3
        drone
         */
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
                "perp_dw",
                X_MULTIPLIER,
                Y_MULTIPLIER,
                FORWARD_OFFSET,
                LATERAL_DISTANCE,
                DW_GEAR_RATIO,
                DW_WHEEL_RADIUS,
                DW_TICKS_PER_REV
        );

        drive.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        ((TrackingWheelLocalizer)drive.getLocalizer()).rightEncoder.setDirection(Encoder.Direction.REVERSE);
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
            purpleServo.setPosition(1);
        } else {
            purpleServo.setPosition(0.5);
        }
    }


    public static double PLANE_START_ANGLE = 0.6;
    public static double PLANE_LAUNCH_ANGLE = 0.80;
    public static double PLANE_FOR_INCR = 0.001;
    public static int PLANE_DELAY = 10;
    @Override
    public void launchPlane() {
        for (double i=PLANE_START_ANGLE; i<PLANE_LAUNCH_ANGLE; i+=PLANE_FOR_INCR) {
            planeAngleServo.setPosition(i);
            GlobalOpMode.opMode.sleep(PLANE_DELAY);
        }
        planeReleaseServo.setPower(0.1);
    }

    @Override
    public TrajectoryDrive getDrive() {
        return drive;
    }


    public int shoulderOffset = 0;

    public void recalibrateShoulder() {
        // Raise the arm up
        shoulderMotor.setTargetPosition(500);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderMotor.setPower(1);
        while (shoulderMotor.isBusy() && !GlobalOpMode.opMode.isStopRequested()) {}
        // Drop the arm
        shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderMotor.setPower(-0.1);
        // Wait for the magnet limit switch to be hit
        while (!magneticLimitSwitch.isPressed() && !GlobalOpMode.opMode.isStopRequested()) {}
        // Record the position
        shoulderOffset = shoulderMotor.getCurrentPosition();

        shoulderMotor.setPower(0);
    }

    public int getShoulderCurrentPosition() {
        return shoulderMotor.getCurrentPosition() - shoulderOffset;
    }
    public void setShoulderTargetPosition(int position) {
        shoulderMotor.setTargetPosition(position + shoulderOffset);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //if (Math.abs(rrobot.shoulderMotor.getCurrentPosition() - armPos) > SAFETY_ARM_DELTA) {
        if (position < shoulderMotor.getCurrentPosition() && position < 1000) {
            shoulderMotor.setPower(0.1);
        } else {
            shoulderMotor.setPower(1);
        }
    }

//    public void extendIntake(boolean state) {
//        if (state) {
//            intakeServo.setAdjustedPosition(-1350, 0.1);
//        } else {
//            intakeServo.setAdjustedPosition(300, 0.1);
//        }
//    }

//    public void initializeIntakeSystem() {
//        wristServo.setPosition(0.5);
//        GlobalOpMode.opMode.sleep(1000);
//        armMotor.setTargetPosition(100);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(0.1);
//        while (armMotor.getCurrentPosition() < 90) {
//            GlobalOpMode.opMode.sleep(100);
//        }
//        //extendIntake(true);
//        //intakeServo.setAdjustedPosition(-1350, 0.1);
//    }
//
//    public void readyForIntake() {
//        intakeServo.setAdjustedPosition(-1350, 0.1);
//        armMotor.setTargetPosition(-240);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(0.1);
//        while (armMotor.getCurrentPosition() > -230) {
//            GlobalOpMode.opMode.sleep(100);
//        }
//        wristServo.setPosition(0.85);
//        intakeServo.setAdjustedPosition(-950, 0.1);
//    }
//
//    public void dropOff() {
//        intakeServo.setAdjustedPosition(-1350, 0.1);
//        armMotor.setTargetPosition(100);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(0.1);
//        while (armMotor.getCurrentPosition() < 90) {
//            GlobalOpMode.opMode.sleep(100);
//        }
//        wristServo.setPosition(0.5);
//        // Raise the arm the rest of the way around
//        armMotor.setTargetPosition(300);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(0.1);
//        while (armMotor.getCurrentPosition() < 290) {
//            GlobalOpMode.opMode.sleep(100);
//        }
//        wristServo.setPosition(0);
//    }
}
