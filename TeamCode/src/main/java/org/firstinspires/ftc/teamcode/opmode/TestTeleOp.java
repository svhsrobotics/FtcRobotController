package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.PsiBot;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.RoboticaBot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
@Config
public class TestTeleOp extends LinearOpMode {
    enum ArmState {
        RAISED,
        PREP,
        PICKUP,
        NEUTRAL,
        MANUAL
    }

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        Robot robot = Robot.thisRobot(hardwareMap);
        //AprilTagLocalizer localizer = new AprilTagLocalizer(robot.getCameras());
        TrajectoryDrive drive = robot.getDrive();
        Configuration config = Configurator.load();

        waitForStart();

        double purplePose = 0.5;
        int armPos = 0;
        double wristPos = 0;
        Toggle pinchToggle = new Toggle();
        ArmState currentState = ArmState.MANUAL;
        int offset = 0;
        double pinchLocation = 0.0;

        while (opModeIsActive()) {
            drive.update(); // MUST be called every loop cycle so that RoadRunner calculates the pose correctly
            Pose2d poseEstimate = drive.getPoseEstimate();

            if (config.fieldCentric) {
                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of that heading
                Vector2d input = new Vector2d(-gamepad1.left_stick_y * 0.5, -gamepad1.left_stick_x * 0.5).rotated(-poseEstimate.getHeading());

                // Pass in the rotated input + right stick value for rotation
                // Rotation is not part of the rotated input thus must be passed in separately
                drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x * 0.5));

                if (gamepad1.x) {
                    drive.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), 0));
                }
            } else {
                drive.setWeightedDrivePower(new Pose2d((-gamepad1.left_stick_y * 0.5) + (-gamepad2.left_stick_y * 0.5), (-gamepad1.left_stick_x * 0.5) + (-gamepad2.left_stick_x * 0.5), (-gamepad1.right_stick_x * 0.5) + (-gamepad2.right_stick_x * 0.5)));
            }

            telemetry.addData("Robot", robot.getClass().toString());
            telemetry.addData("Field Centric", config.fieldCentric);

            telemetry.addData("Current Quadrant", drive.currentQuadrant().toString());
            telemetry.addData("Gamepad 2 is for ENDGAME", "");
            android.util.Log.i("TELEOP", "LOOP2");


            //Pose2d pose = localizer.estimateRobotPoseFromAprilTags(robot.getPrimaryCamera());
            //telemetry.addData("AT Pose", pose);

            if (gamepad2.a) {
                robot.launchPlane();
            }

            if (robot.getClass() == PsiBot.class) {
                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    purplePose = purplePose + 0.1;
                } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    purplePose = purplePose - 0.1;
                }
                if (purplePose > 1) purplePose = 1;
                if (purplePose < 0) purplePose = 0;
                ((PsiBot) robot).planeServo.setPosition(purplePose);
                //  telemetry.addData("Plane Pose", purplePose);
                ((PsiBot) robot).armMotor.setPower((gamepad1.left_trigger * 0.1) - ((gamepad1.right_trigger * 0.1) + (gamepad2.left_trigger * 0.1) - ((gamepad2.right_trigger * 0.1))));
                //  telemetry.addData("Arm location", ((PsiBot) robot).armMotor.getCurrentPosition());
                if (gamepad1.b || gamepad2.b) {
                    ((PsiBot) robot).mosaicServo.setPosition(0);
                } else if (gamepad1.a || gamepad2.a) {
                    ((PsiBot) robot).mosaicServo.setPosition(1);
                }
            } else if (robot.getClass() == RoboticaBot.class) {
                RoboticaBot rrobot = (RoboticaBot) robot;

                // WRIST
                // Just lock wrist twist
                rrobot.wristTwistServo.setPosition(0.5);
                if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    wristPos -= 0.008; // UP
                } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    wristPos += 0.008; // DOWN
                }
                if (wristPos > 1) wristPos = 1;
                if (wristPos < 0) wristPos = 0;
                rrobot.wristLiftServo.setPosition(wristPos);
                telemetry.addData("WRIST", wristPos);
                telemetry.addData("ARM", armPos);
                telemetry.addData("STATE", currentState);

                // SHOULDER : WILL BE REPLACED AFTER WORM GEAR?
                if (gamepad1.left_trigger + gamepad1.right_trigger + gamepad2.right_trigger + gamepad2.left_trigger > 0.05) {
                    armPos = rrobot.getShoulderCurrentPosition() - (int) (gamepad1.left_trigger * 50) - (int) (gamepad2.left_trigger * 50) + (int) (gamepad1.right_trigger * 50) + (int) (gamepad2.right_trigger * 50);
                    currentState = ArmState.MANUAL;
                }

                // If there is a big difference in positions, turn down the power
                rrobot.setShoulderTargetPosition(armPos);
//                rrobot.shoulderMotor.setTargetPosition(armPos);
//                rrobot.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                //if (Math.abs(rrobot.shoulderMotor.getCurrentPosition() - armPos) > SAFETY_ARM_DELTA) {
//                if (armPos < rrobot.shoulderMotor.getCurrentPosition() && armPos < 1000) {
//                    rrobot.shoulderMotor.setPower(SAFETY_ARM_POWER);
//                } else {
//                    rrobot.shoulderMotor.setPower(1);
//                }

                // ELBOW
                if (gamepad1.dpad_left || gamepad2.dpad_left) {
                    rrobot.elbowServo.innerServo.setPower(1.0);
                } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                    rrobot.elbowServo.innerServo.setPower(-1.0);
                } else {
                    rrobot.elbowServo.innerServo.setPower(0.0);
                }

                if (gamepad1.dpad_up || gamepad2.dpad_up) {
                    pinchLocation = pinchLocation + 0.008;
                } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                    pinchLocation -= 0.008;
                }
                if (pinchLocation > 1) {
                    pinchLocation = 1;
                } else if (pinchLocation < 0) {
                    pinchLocation = 0;
                }
                rrobot.pinchServo.setPosition(pinchLocation);


                // HANG
                if (gamepad2.y) {
                    rrobot.hangMotor.setPower(1.0);
                    rrobot.planeAngleServo.setPosition(0.3);
                } else if (gamepad2.x) {
                    rrobot.hangMotor.setPower(-1.0);
                } else {
                    rrobot.hangMotor.setPower(0.0);
                }


                if (gamepad1.y) { // RAISED
                    currentState = ArmState.RAISED;
                    armPos = RAISED_ARM;
                    wristPos = RAISED_WRIST;
                } else if (gamepad1.x) { // NEUTRAL
                    currentState = ArmState.NEUTRAL;
                    armPos = NEUTRAL_ARM;
                    wristPos = NEUTRAL_WRIST;
                } else if (gamepad1.b) { // PREP
                    currentState = ArmState.PREP;
                    armPos = PREP_ARM;
                    wristPos = PREP_WRIST;
                } else if (gamepad1.a) { // PICKUP
                    currentState = ArmState.PICKUP;
                    armPos = PICKUP_ARM;
                    wristPos = PICKUP_WRIST;
                }


                telemetry.addData("OFFSET ", rrobot.shoulderOffset);
                if (gamepad1.back) {
                    rrobot.recalibrateShoulder();
                }

            }

            telemetry.update();
        }
    }

    //public static int ARM_OFFSET = 94;

    public static int RAISED_ARM = 1671;
    public static double RAISED_WRIST = 0.4;

    public static int NEUTRAL_ARM = 198;
    public static double NEUTRAL_WRIST = 0.544;

    public static int PREP_ARM = -7;
    public static double PREP_WRIST = 0.472;

    public static int PICKUP_ARM = -116;
    public static double PICKUP_WRIST = 0.472;
}
