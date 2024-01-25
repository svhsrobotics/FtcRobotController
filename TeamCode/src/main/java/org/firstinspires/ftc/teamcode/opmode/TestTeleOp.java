package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TestBot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;

@TeleOp
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        Robot robot = new TestBot(hardwareMap);
        TrajectoryDrive drive = robot.getDrive();
        Configuration config = Configurator.load();

        waitForStart();

        while (opModeIsActive()) {
            drive.update(); // MUST be called every loop cycle so that RoadRunner calculates the pose correctly
            // Read pose
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

            telemetry.update();
        }
    }
}
