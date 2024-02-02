package org.firstinspires.ftc.teamcode.opmode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.opmode.components.Component;
import org.firstinspires.ftc.teamcode.opmode.components.GoToBoard;
import org.firstinspires.ftc.teamcode.opmode.components.ParkingIn;
import org.firstinspires.ftc.teamcode.opmode.components.ParkingOut;
import org.firstinspires.ftc.teamcode.opmode.components.PurplePixelComponent;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.util.Timeout;
import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@Autonomous
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

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        Robot robot = Robot.thisRobot(hardwareMap);

        //Pose2d startPose = new Pose2d(12,-62, Math.toRadians(90)); // RED_BOARD
        //Pose2d startPose = new Pose2d(-36,-62, Math.toRadians(270)); // RED_AUDIENCE
        //Pose2d startPose = new Pose2d(12,62, Math.toRadians(90)); // BLUE_BOARD
        //Pose2d startPose = new Pose2d(-36,62, Math.toRadians(90)); // BLUE_AUDIENCE

        // Search for AprilTags across the three cameras, until we find one or init ends
        AprilTagLocalizer aprilTag = new AprilTagLocalizer(robot.getCameras());
        Pose2d startPose = null;
        while(opModeInInit() && startPose == null) {
            Log.i("AUTO", "Waiting for AprilTag detection...");
            startPose = estimateWithAllCameras(robot.getCameras(), aprilTag);
        }

        if (startPose != null) {
            Log.i("AUTO", "Found AprilTag, starting Tensorflow");
            telemetry.log().add("Found AprilTag");
        }

        //waitForStart();

        // We didn't find one in init... try once more in start, then give up
        if (startPose == null) {
            // Check one more time
            Log.w("AUTO", "Did not find AprilTag in init, trying one last time");
            startPose = estimateWithAllCameras(robot.getCameras(), aprilTag);
        }

        // Can't find any AprilTags... guess wildly
        if (startPose == null) {
            telemetry.log().add("APRILTAG NOT DETECTED");
            return;
        }

        robot.getDrive().setPoseEstimate(startPose);

        // If we found an AprilTag, then close down the AprilTag Localizer and look for the prop
        TensorFlowDetection.PropPosition tensorPos;
        //Log.i("AUTO", "Found AprilTag, starting Tensorflow");
        telemetry.log().add("Starting TensorFlow");
        aprilTag.close();
        // use the center camera
        TensorFlowDetection tensor = new TensorFlowDetection(robot.getPrimaryCamera().webcamName);
        tensorPos = tensor.getPropPosition(new Timeout(10));
        telemetry.log().add("Tensorflow detected: " + tensorPos);
        if (tensorPos == null) {
            telemetry.log().add("Unable to detect prop");
            tensorPos = TensorFlowDetection.PropPosition.CENTER;
        }

        waitForStart(); // INCORRECT
        if (isStopRequested()) return;

        List<Component> componentList = new ArrayList<>();
        Configuration config = Configurator.load();
        if (config.placePixel) {
            componentList.add(new PurplePixelComponent(robot, tensorPos));
        }
        if (Objects.equals(config.park, "outer")) {
            componentList.add(new ParkingOut(robot));
        } else if (Objects.equals(config.park, "inner")) {
            componentList.add(new ParkingIn(robot));
        } else  if (Objects.equals(config.park, "board")) {
            componentList.add(new GoToBoard(robot, TensorFlowDetection.PropPosition.LEFT));
        }

        for (Component component : componentList) {
            component.drive();
        }

    }




}
