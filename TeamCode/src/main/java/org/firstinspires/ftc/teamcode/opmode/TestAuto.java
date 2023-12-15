package org.firstinspires.ftc.teamcode.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.apache.commons.math3.stat.inference.OneWayAnova;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.testbot.TestBotDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.util.Timeout;
import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.PropellerDetection1;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;
import org.tensorflow.lite.Tensor;

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
        TestBotDrive drive = new TestBotDrive(hardwareMap);

        AprilTagCamera[] cameras = new AprilTagCamera[3];
        cameras[0] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Left"), 8, Math.toRadians(LEFTSEVENTY), Math.toRadians(LEFTFORTYFIVE));
        cameras[1] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Center"), 7, Math.toRadians(90), Math.toRadians(0));
        cameras[2] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Right"), 8, Math.toRadians(RIGHTSEVENTY), Math.toRadians(RIGHTFORTYFIVE));


        //TensorFlowDetection tensor = new TensorFlowDetection(cameras[1].webcamName);
        TensorFlowDetection tensor = null;
        Log.i("PROGRESS", "I did this!");
//        while (!isStopRequested()) {
//            tensor.getPropPosition();
//          android.util.Log.w("LOCATION", "tensor result " + tensor.getPropPosition());
//        }




//        TensorFlowDetection.PropPosition tensorPos = TensorFlowDetection.PropPosition.LEFT;



        AprilTagLocalizer aprilTag = new AprilTagLocalizer(cameras);
        Pose2d startPose = null;
        Log.i("PROGRESS", "here?");
        while(opModeInInit() && !isStopRequested() ) {
            Log.i("PROGRESS", "before");
            startPose = estimateWithAllCameras(cameras, aprilTag);
            Log.i("PROGRESS", "after");
            if (startPose != null) {
                drive.setPoseEstimate(startPose);
                drive.update();
            }
        }
        waitForStart();
        Log.i("PROGRESS", "Maybe?");
        if (startPose == null) {
            // Check one more time
            Log.w("APRILTAG", "Did not find AprilTag in init, trying again");
            startPose = estimateWithAllCameras(cameras, aprilTag);
        }

        if (startPose == null) {
            // Give up
            //startPose = new Pose2d(0, 0, 0);
            throw new RuntimeException("Could not find AprilTag to determine start pose");
        }
        Log.i("PROGRESS", "may as well check");
        TrajectorySequence traj = null;

        drive.setPoseEstimate(startPose);
        Log.i("PROGRESS", "Im here too!");

        TensorFlowDetection.PropPosition tensorPos = tensor.getPropPosition();
        if (tensorPos == null) {
            tensorPos = TensorFlowDetection.PropPosition.CENTER;
        }


        switch (AprilTagLocalizer.whichQuadrant(startPose)) {
            case RED_BOARD:
                switch (tensorPos) {
                    case LEFT:
                    traj = drive.trajectorySequenceBuilder(startPose)
                            .lineTo(new Vector2d(10, -36))
                            .turn(Math.toRadians(90))

                            .addTemporalMarker(() ->{
                                //ARM DROPS PIXEL
                                android.util.Log.i("DROP", "Drop");

                            })

                            .splineToSplineHeading(new Pose2d(11, -36, Math.toRadians(180)), Math.toRadians(180))

                            .turn(Math.toRadians(180))

                            .splineToSplineHeading(new Pose2d(3 * 12 + 5, -3 * 12, 0), 0)
                            .turn(Math.toRadians(90))
                            .lineTo(new Vector2d(startPose.getX(), startPose.getY()))
                            .build();
                    break;
                    case RIGHT:
                        traj = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(16, -36))
                            .turn(Math.toRadians(-90))
                            .addTemporalMarker(() ->{
                                //ARM DROPS PIXEL
                                Log.i("DROP", "drop");
                            })
                                .lineTo(new Vector2d(startPose.getX(), startPose.getY()))
                            .splineToSplineHeading(new Pose2d(3 * 12 + 5, -3 * 12, 0), 0)
                            .build();
                    break;
                    case CENTER:

                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(16, -36))
                                .addTemporalMarker(() -> {
                                    //ARM DROPS PIXEL
                                    Log.i("DROP", "drop");

                                })
                                .turn(Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(3 * 12 + 5, -3 * 12, 0), 0)
                                .build();
                    break;
                }
            break;
            case RED_AUDIENCE:
                switch (tensorPos) {
                    case LEFT:
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
                            .lineToSplineHeading(new Pose2d(2 * 12, -3 * 12 + 2, Math.toRadians(90)))
                            .lineToSplineHeading(new Pose2d(3 * 12 + 5, -3 * 12 + 2, 0))
                            .build();

                    break;
                    case RIGHT:
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), -3 * 12 + 2)) // drive forward to prevent the spline from cutting through the poles
                                .turn(Math.toRadians(-90))
                                .addDisplacementMarker(() -> {
                                    //PIXEL DROP
                                    Log.i("DROP", "drop");
                                })
                                .lineTo(new Vector2d(startPose.getX(), startPose.getY()))
                                //.splineToSplineHeading(new Pose2d(3*12 + 5, -3*12, 0), 0)
                                .lineToSplineHeading(new Pose2d(2 * 12, -3 * 12 + 2, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(3 * 12 + 5, -3 * 12 + 2, 0))
                                .build();
                    break;
                    case CENTER:
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), -3 * 12 + 2)) // drive forward to prevent the spline from cutting through the poles
                                .addTemporalMarker(() -> {
                                    //PIXEL DROP
                                    Log.i("DROP", "drop");
                                })
                                .turn(Math.toRadians(90))
                                //.splineToSplineHeading(new Pose2d(3*12 + 5, -3*12, 0), 0)
                                .lineToSplineHeading(new Pose2d(2 * 12, -3 * 12 + 2, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(3 * 12 + 5, -3 * 12 + 2, 0))
                                .build();
                    break;
                }
            break;
            case BLUE_AUDIENCE:
                switch (tensorPos) {
                    case LEFT:
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), 3*12-2))
                                .turn(Math.toRadians(90))
                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    Log.i("ARM", "drop");
                                })
                                .lineToSplineHeading(new Pose2d(2 * 12, 3 * 12 + 2, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(3 * 12 + 5, 3 * 12 + 2, 0))
                                .build();


                    break;
                    case RIGHT:
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), 3*12-2))
                                .turn(Math.toRadians(-90))
                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    Log.i("DROP", "drop");
                                })
                                .lineTo(new Vector2d(startPose.getX(),startPose.getY()))
                                .turn(Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(2 * 12, 3 * 12 + 2, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(3 * 12 + 5, 3 * 12 + 2, 0))
                                .build();
                    break;
                    case CENTER:
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), 3*12-2))
                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    //wait(200);
                                    Log.i("DROP", "drop");
                                })
                                .turn(Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(2 * 12, 3 * 12 + 2, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(3 * 12 + 5, 3 * 12 + 2, 0))
                                .build();
                    break;


                }
            break;


            case BLUE_BOARD:
                switch (tensorPos) {
                    case LEFT:
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), 36))
                                .turn(Math.toRadians(90))
                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    Log.i("DROP", "drop");
                                })

                                .splineToSplineHeading(new Pose2d(3 * 12 + 5, 3 * 12, 0), 0)
                                .build();
                    break;
                    case RIGHT:
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), 36))
                                .turn(Math.toRadians(-90))
                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    Log.i("DROP", "drop");
                                })
                                .lineTo(new Vector2d(startPose.getX(),startPose.getY()))
                                .turn(Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(3 * 12 + 5, 3 * 12, 0), 0)
                                .build();
                    break;
                    case CENTER:
                        traj = drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(startPose.getX(), 36))

                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    Log.i("DROP", "drop");
                                })
                                .turn(Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(3 * 12 + 5, 3 * 12, 0), 0)
                                .build();
                    break;
                }
            break;
        }
        Log.i("PROGRESS", "I got here???");

        if (!isStopRequested())
            drive.followTrajectorySequence(traj);

        //aprilTag.centerOnTag(cameras[1], drive, 5);

        Log.i("PROGRESS", "whatttttt");
    }
}




