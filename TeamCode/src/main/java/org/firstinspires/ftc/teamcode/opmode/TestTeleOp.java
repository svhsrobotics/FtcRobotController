package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.PsiBot;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.RoboticaBot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
public class TestTeleOp extends LinearOpMode {
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
        double wristPos = 1;
        Toggle pinchToggle = new Toggle();

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

            telemetry.addData("Robot", robot.getClass().toString());
            telemetry.addData("Field Centric", config.fieldCentric);

            telemetry.addData("Current Quadrant", drive.currentQuadrant().toString());
            android.util.Log.i("TELEOP", "LOOP2");


            //Pose2d pose = localizer.estimateRobotPoseFromAprilTags(robot.getPrimaryCamera());
            //telemetry.addData("AT Pose", pose);

            if (gamepad1.a) {
                robot.launchPlane();
            }

            if (robot.getClass() == PsiBot.class) {
                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    purplePose = purplePose + 0.1;
                } else if (gamepad1.right_bumper|| gamepad2.right_bumper) {
                    purplePose = purplePose - 0.1;
                }
                if (purplePose > 1) purplePose = 1;
                if (purplePose < 0) purplePose = 0;
                ((PsiBot) robot).planeServo.setPosition(purplePose);
              //  telemetry.addData("Plane Pose", purplePose);
                ((PsiBot) robot).armMotor.setPower((gamepad1.left_trigger*0.1)-((gamepad1.right_trigger*0.1) + (gamepad2.left_trigger*0.1)-((gamepad2.right_trigger*0.1))));
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
                wristPos += gamepad1.right_stick_y * .008;
                if (wristPos > 1) wristPos = 1;
                if (wristPos < 0) wristPos = 0;
                rrobot.wristLiftServo.setPosition(wristPos);
                telemetry.addData("WRIST", wristPos);

                // SHOULDER : WILL BE REPLACED AFTER WORM GEAR?
                if (gamepad1.left_trigger + gamepad1.right_trigger > 0.05) {
                    armPos = rrobot.shoulderMotor.getCurrentPosition() + (int)(gamepad1.left_trigger * 50) - (int)(gamepad1.right_trigger * 50);
                }

                rrobot.shoulderMotor.setTargetPosition(armPos);
                rrobot.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rrobot.shoulderMotor.setPower(1);

                // ELBOW : JUST KICK FOR NOW, REPLACE WITH ANALOG WITH MORE POWER
                if (gamepad1.y) {
                    rrobot.elbowServo.innerServo.setPower(-0.3);
                } else {
                    rrobot.elbowServo.innerServo.setPower(0);
                }

                // PINCH
                if (pinchToggle.update(gamepad1.x)) {
                    rrobot.pinchServo.setPosition(1);
                } else {
                    rrobot.pinchServo.setPosition(0);
                }
            }

            telemetry.update();
        }
    }
}
