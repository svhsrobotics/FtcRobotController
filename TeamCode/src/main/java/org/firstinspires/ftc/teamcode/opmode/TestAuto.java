package org.firstinspires.ftc.teamcode.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.panthera.PantheraDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;

@Autonomous
@Config
public class TestAuto extends LinearOpMode {
    private Pose2d estimateWithAllCameras(AprilTagCamera[] cameras, AprilTagLocalizer aprilTag) {
        android.util.Log.i("PROGRESS", "log1");
        Pose2d pose = null;
        android.util.Log.i("PROGRESS", "log2");
        for (AprilTagCamera camera : cameras) {
            android.util.Log.i("PROGRESS", "trying to get pose");
            pose = aprilTag.estimateRobotPoseFromAprilTags(camera);
            if (pose != null) {
                android.util.Log.i("PROGRESS", "break");
                break;
            }
        }
        android.util.Log.i("PROGRESS", "returning");
        return pose;
    }




    public static double LEFTFORTYFIVE = -45;
    public static double LEFTSEVENTY = 70;
    public static double RIGHTFORTYFIVE = 45;
    public static double RIGHTSEVENTY = -70;

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        PantheraDrive drive = new PantheraDrive(hardwareMap);

        AprilTagCamera[] cameras = new AprilTagCamera[3];
        cameras[0] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Left"), 8, Math.toRadians(LEFTSEVENTY), Math.toRadians(LEFTFORTYFIVE));
        cameras[1] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Center"), 7, Math.toRadians(90), Math.toRadians(0));
        cameras[2] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Right"), 8, Math.toRadians(RIGHTSEVENTY), Math.toRadians(RIGHTFORTYFIVE));

        Servo purpleServo = hardwareMap.get(Servo.class, "purple");

        // Search for AprilTags across the three cameras, until we find one or init ends
        AprilTagLocalizer aprilTag = new AprilTagLocalizer(cameras);
        Pose2d startPose = null;
        while(opModeInInit() && startPose == null) {
            Log.i("AUTO", "Waiting for AprilTag detection...");
            startPose = estimateWithAllCameras(cameras, aprilTag);
        }

        // If we found an AprilTag, then close down the AprilTag Localizer and look for the prop
        TensorFlowDetection.PropPosition tensorPos = TensorFlowDetection.PropPosition.CENTER;
        if (startPose != null) {
            Log.i("AUTO", "Found AprilTag, starting Tensorflow");
            telemetry.log().add("Found AprilTag");
            aprilTag.close();
            // use the center camera
            TensorFlowDetection tensor = new TensorFlowDetection(cameras[1].webcamName);
            tensorPos = tensor.getPropPosition();
            telemetry.log().add("Tensorflow detected: " + tensorPos);
            if (tensorPos == null) {
                telemetry.log().add("Unable to detect prop");
                tensorPos = TensorFlowDetection.PropPosition.CENTER;
            }
        } else {
            telemetry.log().add("Didn't see AprilTag in init");
            Log.w("AUTO", "Didn't see AprilTag in init, so we didn't look for the prop");
        }

        waitForStart();

        // We didn't find one in init... try once more in start, then give up
        if (startPose == null) {
            // Check one more time
            Log.w("AUTO", "Did not find AprilTag in init, trying one last time");
            startPose = estimateWithAllCameras(cameras, aprilTag);
        }

        // Can't find any AprilTags... guess wildly
        if (startPose == null) {
            telemetry.log().add("APRILTAG NOT DETECTED");
            return;
        }

        TrajectorySequence traj = null;

        // Tell RoadRunner about the pose we got from the AprilTags
        startPose = new Pose2d(startPose.getX(), startPose.getY(), startPose.getHeading() + Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        switch (AprilTagLocalizer.whichQuadrant(startPose)) {
            case RED_BOARD:
                switch (tensorPos) {
                    case LEFT: // TODO
                    traj = drive.trajectorySequenceBuilder(startPose)
                            .lineTo(new Vector2d(10, -36))
                            .turn(Math.toRadians(90))
                            .addTemporalMarker(() ->{
                                android.util.Log.i("DROP", "Drop");
                                purpleServo.setPosition(1);
                            })


                            .turn(Math.toRadians(180))

                            .splineTo(new Vector2d(3 * 12 + 5, -3 * 12), 0)
                            .turn(Math.toRadians(90))
                            .lineTo(new Vector2d(startPose.getX(), startPose.getY()))
                            .build();
                    break;
                    case RIGHT:// TODO
                        traj = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(16, -36))
                            .turn(Math.toRadians(-90))
                            .addTemporalMarker(() ->{
                                //ARM DROPS PIXEL
                                Log.i("DROP", "drop");
                                purpleServo.setPosition(1);
                            })
                                .lineTo(new Vector2d(startPose.getX(), startPose.getY()))
                            .splineTo(new Vector2d(3 * 12 + 5, -3 * 12), 0)
                            .build();
                    break;
                    case CENTER: // TODO

                        // MOVED TO BLUE
                    break;
                }
            break;
            case RED_AUDIENCE:
                switch (tensorPos) {
                    case LEFT: // TODO
                    traj = drive.trajectorySequenceBuilder(startPose)
                            .lineTo(new Vector2d(-34, -3 * 12 + 2)) // drive forward to prevent the spline from cutting through the poles

                            .turn(Math.toRadians(90))

                            .addTemporalMarker(() -> {
                                //PIXEL DROP

                                Log.i("DROP", "drop");

                            })
                            .waitSeconds(1)
                            .turn(Math.toRadians(-180))

                            //.splineToSplineHeading(new Pose2d(3*12 + 5, -3*12, 0), 0)
                            .lineToSplineHeading(new Pose2d(2 * 12, -3 * 12 + 2, Math.toRadians(0)))
                            .lineToSplineHeading(new Pose2d(3 * 12 + 5, -3 * 12 + 2, 0))
                            .build();

                    break;
                    case RIGHT: // TODO
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), -3 * 12 + 2))
                                .turn(Math.toRadians(-90))
                                .addDisplacementMarker(() -> {
                                    //PIXEL DROP
                                    Log.i("DROP", "drop");
                                })
                                .lineTo(new Vector2d(startPose.getX(), startPose.getY()))
                                //.splineToSplineHeading(new Pose2d(3*12 + 5, -3*12, 0), 0)
                                .lineToSplineHeading(new Pose2d(2 * 12, -3 * 12 + 2, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(3 * 12 + 5, -3 * 12 + 2, 0))
                                .build();
                    break;
                    case CENTER:
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), -2*12 - 3))
                                .addTemporalMarker(() -> {
                                    Log.i("DROP", "dropping purple");
                                    purpleServo.setPosition(1);
                                })
                                .waitSeconds(1)
                                .lineTo(new Vector2d(startPose.getX(), -5 * 12+3))
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(3*12+7, -5*12+4))
                                .build();
                    break;
                }
            break;
            case BLUE_AUDIENCE:
                switch (tensorPos) {
                    case LEFT: // TODO
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), 3*12-2))
                                .turn(Math.toRadians(90))
                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    Log.i("ARM", "drop");
                                })
                                .lineToSplineHeading(new Pose2d(2 * 12, 3 * 12 + 2, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(3 * 12 + 5, 3 * 12 + 2, 0))
                                .build();


                    break;
                    case RIGHT: // TODO
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), 3*12-2))
                                .turn(Math.toRadians(-90))
                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    Log.i("DROP", "drop");
                                })
                                .lineTo(new Vector2d(startPose.getX(),startPose.getY()))
                                .turn(Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(2 * 12, 3 * 12 + 2, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(3 * 12 + 5, 3 * 12 + 2, 0))
                                .build();
                    break;
                    case CENTER: // TODO
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), 3*12-2))
                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    //wait(200);
                                    Log.i("DROP", "drop");
                                })
                                .turn(Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(2 * 12, 3 * 12 + 2, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(3 * 12 + 5, 3 * 12 + 2, 0))
                                .build();
                    break;


                }
            break;


            case BLUE_BOARD:
                switch (tensorPos) {
                    case LEFT: // TODO
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), 36))
                                .turn(Math.toRadians(90))
                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    Log.i("DROP", "drop");
                                })

                                .splineTo(new Vector2d(3 * 12 + 5, 3 * 12), 0)
                                .build();
                    break;
                    case RIGHT:
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), 3*12-2))
                                .turn(Math.toRadians(-90))

                                .lineTo(new Vector2d(1, 3*12-2))
                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    Log.i("DROP", "dropping purple");
                                    purpleServo.setPosition(1);
                                })
                                .lineTo(new Vector2d(3*12+7, 3*12-2))
                                .build();
                    break;
                    case CENTER:
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), 26))
                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    Log.i("DROP", "dropping purple");
                                    purpleServo.setPosition(1);
                                })
                                .lineTo(new Vector2d(startPose.getX(), 36))
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(3*12+7, 3*12))
                                .build();
                    break;
                }
            break;
        }

        if (!isStopRequested())
            drive.followTrajectorySequence(traj);

        //aprilTag.centerOnTag(cameras[1], drive, 5);

    }
}




