package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.PsiBot;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.RoboticaBot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.util.Debouncer;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        Robot robot = Robot.thisRobot(hardwareMap);
        AprilTagLocalizer localizer = new AprilTagLocalizer(robot.getCameras());
        TrajectoryDrive drive = robot.getDrive();
        Configuration config = Configurator.load();


        waitForStart();

        if (robot.getClass() == RoboticaBot.class) {
            ///((RoboticaBot) robot).extendIntake(true);
            ((RoboticaBot) robot).initializeIntakeSystem();
        }

        double wristPose = 0;
        Debouncer iDebouncer = new Debouncer();
        double purplePose = .5;

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
                drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y * 0.5, -gamepad1.left_stick_x * 0.5, -gamepad1.right_stick_x * 0.5));
            }

            telemetry.addData("Robot", robot.getClass().getName());
            telemetry.addData("Field Centric", config.fieldCentric);

            telemetry.addData("Current Quadrant", drive.currentQuadrant().toString());

            //Pose2d pose = localizer.estimateRobotPoseFromAprilTags(robot.getPrimaryCamera());
            //telemetry.addData("AT Pose", pose);

            if (robot.getClass() == PsiBot.class) {

                if (gamepad1.left_bumper) {
                    purplePose = purplePose + 0.1;
                } else if (gamepad1.right_bumper) {
                    purplePose = purplePose - 0.1;
                }
                if (purplePose > 1) purplePose = 1;
                if (purplePose < 0) purplePose = 0;
                ((PsiBot) robot).planeServo.setPosition(purplePose);
                telemetry.addData("Plane Pose", purplePose);
                ((PsiBot) robot).armMotor.setPower((gamepad1.left_trigger*0.1)-((gamepad1.right_trigger*0.1)));
                telemetry.addData("Arm location", ((PsiBot) robot).armMotor.getCurrentPosition());
               if (gamepad1.b) {
                   ((PsiBot) robot).mosaicServo.setPosition(0);
               } else if (gamepad1.a) {
                   ((PsiBot) robot).mosaicServo.setPosition(1);
               }
            } else if (robot.getClass() == RoboticaBot.class) {
                RoboticaBot rrobot = (RoboticaBot) robot;
                if (iDebouncer.update(gamepad1.a)) {
                     rrobot.readyForIntake();
                }

                //rrobot.armMotor.setPower((gamepad1.left_trigger*0.5)-((gamepad1.right_trigger*0.5)));

//                if (gamepad1.dpad_up) {
//                    rrobot.intakeMotor.setPower(0.4);
//                } else if (gamepad1.dpad_down) {
//                    rrobot.intakeMotor.setPower(-0.4);
//                } else {
//                    rrobot.intakeMotor.setPower(0);
//                }

//                if (gamepad1.dpad_left) {
//                    wristPose += 0.05;
//                } else if (gamepad1.dpad_right) {
//                    wristPose -= 0.05;
//                }
//                if (wristPose < 0) wristPose = 0;
//                if (wristPose > 1) wristPose = 1;
//                rrobot.wristServo.setPosition(wristPose);
                //telemetry.addData("Wrist Pose", wristPose);

                telemetry.addData("Arm Pose", rrobot.armMotor.getCurrentPosition());
                telemetry.addData("Extend Amount", rrobot.intakeServo.getAdjustedPosition());

            }

            telemetry.update();


        }
    }

    public void centerOnTag() {

    }
}
