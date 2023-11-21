package org.firstinspires.ftc.teamcode.opmode;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.testbot.TestBotDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Blinker;
import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;

import java.util.List;

@Autonomous
@Config
public class TestAuto extends LinearOpMode {
    int[] colors = {
            Color.rgb(255, 0,   0),   // RED
            Color.rgb(255, 128, 0),   // ORANGE
            Color.rgb(255, 255, 0),   // YELLOW
            Color.rgb(128, 255, 0),   // LIME
            Color.rgb(0,   255, 0),   // GREEN
            Color.rgb(0,   255, 128), // MINT
            Color.rgb(0,   255, 255), // CYAN
            Color.rgb(0,   128, 255), // BLUE
            Color.rgb(0,   0,   255), // NAVY
            Color.rgb(128, 0,   255), // PURPLE
            Color.rgb(255, 0,   255), // PINK
            Color.rgb(255, 0,   128), // HOT PINK
    };

    Blinker.Pattern rainbowPattern() {
        Blinker.Pattern pattern = new Blinker.Pattern();
        for (int color : colors) {
            pattern.addStep(color, 150);
        }
        return pattern;
    }

    LynxModule controlHub(List<LynxModule> modules) {
        for (LynxModule module : modules) {
            if (module.getModuleAddress() == LynxConstants.CH_EMBEDDED_MODULE_ADDRESS) {
                return module;
            }
        }
        return null;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        TestBotDrive drive = new TestBotDrive(hardwareMap);

        //List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        //LynxModule chub = controlHub(modules);
        //Blinker blinker = new Blinker(chub);
        //blinker.idle();

        AprilTagCamera[] cameras = new AprilTagCamera[3];
        cameras[0] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Left"), 8, Math.toRadians(70), Math.toRadians(-45));
        cameras[1] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Center"), 7, Math.toRadians(90), Math.toRadians(0));
        cameras[2] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Right"), 8, Math.toRadians(-70), Math.toRadians(45));

        AprilTagLocalizer aprilTag = new AprilTagLocalizer(cameras);

        Pose2d startPose = null;

        waiting:
        while(opModeInInit()) {
            // Try each of the cameras until we get a pose estimate
            for (AprilTagCamera camera : cameras) {
                Pose2d pose = aprilTag.estimateRobotPoseFromAprilTags(camera);
                if (pose != null) {
                    startPose = pose;
                    //blinker.displayPattern(rainbowPattern());
                    break waiting;
                }
            }
        }

        if (startPose == null) {
            // Check one more time
            for (AprilTagCamera camera : cameras) {
                Pose2d pose = aprilTag.estimateRobotPoseFromAprilTags(camera);
                if (pose != null) {
                    startPose = pose;
                    break;
                    //blinker.displayPattern(rainbowPattern())
                }
            }
        }

        if (startPose == null) {
            // Give up
            //startPose = new Pose2d(0, 0, 0);
            throw new RuntimeException("BAD");
        }

        TrajectorySequence traj = null;

        android.util.Log.w("APRILTAG", "POSE1:"  + startPose);
        //startPose = new Pose2d(-38,-62, Math.toRadians(90));
        android.util.Log.w("APRILTAG", "POSE2:" + startPose);

        drive.setPoseEstimate(startPose);

        switch (AprilTagLocalizer.whichQuadrant(startPose)) {
            case RED_AUDIENCE:
            case RED_BOARD:
                // HACK: Fudge the startPose
                //startPose = new Pose2d(startPose.getX(), startPose.getY() + FUDGE, startPose.getHeading());
                //drive.setPoseEstimate(startPose);

                traj = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(startPose.getX(), -24))
                        //.splineTo(new Vector2d(0, 0), 0)
                        //.splineToConstantHeading(new Vector2d(0, 0), 0)
                        .splineToSplineHeading(new Pose2d(0,0,0), 0)
                        //.lineTo(new Vector2d(-34, -36))
                        //.strafeTo(new Vector2d(48, -36))
                        //.splineTo(new Vector2d(48, -35), 0)
                        .build();
                break;
            case BLUE_AUDIENCE:
            case BLUE_BOARD:
                // HACK: Fudge the startPose
                //startPose = new Pose2d(startPose.getX(), startPose.getY(), startPose.getHeading());
                //drive.setPoseEstimate(startPose);

                traj = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d(startPose.getX(), 24))
                        .splineTo(new Vector2d(0, 0), 0)
                        .build();
                break;
        }

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(traj);
    }
}




