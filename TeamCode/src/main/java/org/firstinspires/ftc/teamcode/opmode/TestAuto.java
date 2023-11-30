package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.testbot.TestBotDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.PropellerDetection1;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;

@Autonomous
@Config
public class TestAuto extends LinearOpMode {
    private Pose2d estimateWithAllCameras(AprilTagCamera[] cameras, AprilTagLocalizer aprilTag) {
        Pose2d pose = null;
        for (AprilTagCamera camera : cameras) {
            pose = aprilTag.estimateRobotPoseFromAprilTags(camera);
            if (pose != null) {
                break;
            }
        }
        return pose;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        TestBotDrive drive = new TestBotDrive(hardwareMap);

        AprilTagCamera[] cameras = new AprilTagCamera[3];
        cameras[0] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Left"), 8, Math.toRadians(70), Math.toRadians(-45));
        cameras[1] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Center"), 7, Math.toRadians(90), Math.toRadians(0));
        cameras[2] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Right"), 8, Math.toRadians(-70), Math.toRadians(45));


        TensorFlowDetection tensor = new TensorFlowDetection(cameras[1].webcamName);
        while (!isStopRequested()) {
            android.util.Log.w("LOCATION", "tensor result " + tensor.getPropPosition());
        }
        AprilTagLocalizer aprilTag = new AprilTagLocalizer(cameras);
        Pose2d startPose = null;

        while(opModeInInit() && !isStopRequested() && startPose == null) {
            startPose = estimateWithAllCameras(cameras, aprilTag);
        }

        if (startPose == null) {
            // Check one more time
            android.util.Log.w("APRILTAG", "Did not find AprilTag in init, trying again");
            startPose = estimateWithAllCameras(cameras, aprilTag);
        }

        if (startPose == null) {
            // Give up
            //startPose = new Pose2d(0, 0, 0);
            throw new RuntimeException("Could not find AprilTag to determine start pose");
        }

        TrajectorySequence traj = null;

        drive.setPoseEstimate(startPose);



        switch (AprilTagLocalizer.whichQuadrant(startPose)) {
            case RED_BOARD:
                traj = drive.trajectorySequenceBuilder(startPose)
                        .splineToSplineHeading(new Pose2d(3*12 + 5, -3*12, 0), 0)
                        .build();
                break;
            case RED_AUDIENCE:
                traj = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(startPose.getX(), -3*12 + 2)) // drive forward to prevent the spline from cutting through the poles
                        //.splineToSplineHeading(new Pose2d(3*12 + 5, -3*12, 0), 0)
                        .lineToSplineHeading(new Pose2d(2*12, -3*12 + 2, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(3*12+5, -3*12+2, 0))
                        .build();
                break;
            case BLUE_AUDIENCE:
            case BLUE_BOARD:
                break;
        }

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(traj);

        aprilTag.centerOnTag(cameras[1], drive, 5);
    }
}




