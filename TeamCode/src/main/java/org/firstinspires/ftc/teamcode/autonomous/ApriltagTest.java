package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AprilTag Test")
public class ApriltagTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        WebcamName[] webcams = new WebcamName[3];
        webcams[0] = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcams[1] = hardwareMap.get(WebcamName.class, "Webcam 2");
        webcams[2] = hardwareMap.get(WebcamName.class, "Webcam 3");

        AprilTagLocalizer aprilTag = new AprilTagLocalizer(webcams);

        android.util.Log.i("AprilTag Test", "Waiting for start...");

        while (opModeInInit()) {
            Pose2d robotPose = aprilTag.estimateRobotPoseFromAprilTags(1, 10, Math.toRadians(60));

            android.util.Log.i("AprilTag Test", "Robot pose: " + robotPose);

            if (robotPose != null) {
                drive.setPoseEstimate(robotPose);
            }
            drive.update();
        }

        //switchCamera(0);
        waitForStart();

        while(opModeIsActive()) {
            Pose2d robotPose = aprilTag.estimateRobotPoseFromAprilTags(1, 10, Math.toRadians(60));

            android.util.Log.i("AprilTag Test", "Robot pose: " + robotPose);

            if (robotPose != null) {
                drive.setPoseEstimate(robotPose);
            }
            drive.update();
        }


        aprilTag.close();
    }
}