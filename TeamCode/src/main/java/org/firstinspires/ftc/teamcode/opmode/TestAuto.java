package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.testbot.TestBotDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;

@Autonomous
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TestBotDrive drive = new TestBotDrive(hardwareMap);

        WebcamName[] webcams = new WebcamName[3];
        webcams[0] = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcams[1] = hardwareMap.get(WebcamName.class, "Webcam 2");
        webcams[2] = hardwareMap.get(WebcamName.class, "Webcam 3");

        AprilTagLocalizer aprilTag = new AprilTagLocalizer(webcams);

        Pose2d startPose = new Pose2d(-34, -62, Math.toRadians(90)); // Backup start pose
        while(opModeInInit()) {
            Pose2d pose = aprilTag.estimateRobotPoseFromAprilTags(1, 10, Math.toRadians(70));
            if (pose != null) {
                startPose = pose;
                drive.setPoseEstimate(startPose);
                drive.update();
            }
        }

        TrajectorySequence traj = null;

        switch (aprilTag.whichQuadrant(startPose)) {
            case RED_AUDIENCE:
                // HACK: Fudge the startPose
                startPose = new Pose2d(startPose.getX(), startPose.getY() - 5, startPose.getHeading());
                drive.setPoseEstimate(startPose);

                traj = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(-34, -36))
                        .strafeTo(new Vector2d(48, -36))
                        .build();
                break;
            case RED_BOARD:
                break;
            case BLUE_AUDIENCE:
                break;
            case BLUE_BOARD:
                // HACK: Fudge the startPose
                startPose = new Pose2d(startPose.getX(), startPose.getY() + 5, startPose.getHeading());
                drive.setPoseEstimate(startPose);

                traj = drive.trajectorySequenceBuilder(startPose)
                        .strafeTo(new Vector2d(48, 36))
                        .build();
                break;
        }

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(traj);
    }
}




