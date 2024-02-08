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

        if (robot.getClass() == RoboticaBot.class) {
            ///((RoboticaBot) robot).extendIntake(true);
            ((RoboticaBot) robot).initializeIntakeSystem();
        }

        double wristPose = 0;
        //Debouncer iDebouncer = new Debouncer();
        //Debouncer debouncer2 = new Debouncer();
        double purplePose = .5;
        int armPos = 0;
        if (robot.getClass() == RoboticaBot.class) {
            armPos = ((RoboticaBot) robot).armMotor.getCurrentPosition();
        }

        while (opModeIsActive()) {
            android.util.Log.i("TELEOP", "LOOP");
            //drive.update(); // MUST be called every loop cycle so that RoadRunner calculates the pose correctly
            android.util.Log.i("TELEOP", "LOOPA");

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
                android.util.Log.i("TELEOP", "LOOPB");

            } else {
                drive.setWeightedDrivePower(new Pose2d(-gamepad2.left_stick_y * 0.5, -gamepad2.left_stick_x * 0.5, -gamepad2.right_stick_x * 0.5));
                android.util.Log.i("TELEOP", "LOOPC");

            }

            telemetry.addData("Robot", robot.getClass().getName());
            telemetry.addData("Field Centric", config.fieldCentric);

            telemetry.addData("Current Quadrant", drive.currentQuadrant().toString());
            android.util.Log.i("TELEOP", "LOOP2");


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
                android.util.Log.i("TELEOP", "LOOP2A");
            } else if (robot.getClass() == RoboticaBot.class) {
                RoboticaBot rrobot = (RoboticaBot) robot;

                android.util.Log.i("TELEOP", "LOOP2B");

                if (gamepad1.left_trigger + gamepad1.right_trigger > 0.05) {
                    rrobot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rrobot.armMotor.setVelocity((gamepad1.left_trigger - gamepad1.right_trigger) * 0.4);
                    armPos = rrobot.armMotor.getCurrentPosition();
                } else {
                    //armPos = rrobot.armMotor.getCurrentPosition();
                    rrobot.armMotor.setTargetPosition(armPos);
                    rrobot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rrobot.armMotor.setPower(0.3);
                }

//                rrobot.armMotor.setTargetPosition(armPos);
//                rrobot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rrobot.armMotor.setPower(0.3);


                //rrobot.armMotor.setPower((gamepad1.left_trigger*0.5)-((gamepad1.right_trigger*0.5)));

                if (gamepad1.dpad_up) {
                    rrobot.intakeMotor.setPower(1);
                } else if (gamepad1.dpad_down) {
                    rrobot.intakeMotor.setPower(-1);
                } else {
                    rrobot.intakeMotor.setPower(0);
                }
                android.util.Log.i("TELEOP", "LOOP2C");



                if (gamepad1.left_bumper) {
                    wristPose += 0.05;
                } else if (gamepad1.right_bumper) {
                    wristPose -= 0.05;
                }
                if (wristPose < 0) wristPose = 0;
                if (wristPose > 1) wristPose = 1;

                rrobot.wristServo.setPosition(wristPose);

                double p;
                if (gamepad1.dpad_left) {
                    p = 0.5;
                } else if (gamepad1.dpad_right) {
                    p = -0.5;
                } else {
                    p = 0;
                }

                rrobot.intakeServo.innerServo.setPower(p);

                if (gamepad1.x || gamepad2.x) {
                    rrobot.hangMotor.setPower(1);
                } else if (gamepad1.y || gamepad2.y) {
                    rrobot.hangMotor.setPower(-1);
                } else {
                    rrobot.hangMotor.setPower(0);
                }
                //telemetry.addData("Wrist Pose", wristPose);

                //telemetry.addData("Arm Pose", rrobot.armMotor.getCurrentPosition());
                //telemetry.addData("Extend Amount", rrobot.intakeServo.getAdjustedPosition());

            }
            android.util.Log.i("TELEOP", "LOOPEND");


            telemetry.update();


        }
    }

    public void centerOnTag() {

    }
}
