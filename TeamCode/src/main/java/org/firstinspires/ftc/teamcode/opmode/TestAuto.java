package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.PsiBot;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.RoboticaBot;
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
        Pose2d pose = null;
        for (AprilTagCamera camera : cameras) {
            pose = aprilTag.estimateRobotPoseFromAprilTags(camera);
            if (pose != null) {
                break;
            }
        }
        return pose;
    }

    private TensorFlowDetection.PropPosition detectProp(Robot robot, int timeout) {
        if (robot.getClass() == PsiBot.class) {
            ((PsiBot) robot).armMotor.setTargetPosition(-109);
            ((PsiBot) robot).armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((PsiBot) robot).armMotor.setPower(.1);
        }
        TensorFlowDetection tensor = new TensorFlowDetection(robot.getPrimaryCamera().webcamName);
        TensorFlowDetection.PropPosition position = tensor.getPropPosition(new Timeout(timeout));
        if (position == null) {
            telemetry.log().add("Unable to detect prop, using CENTER");
            position = TensorFlowDetection.PropPosition.CENTER;
        }
        return position;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        Robot robot = Robot.thisRobot(hardwareMap);
        Configuration config = Configurator.load();

        // Set servo position right away so that it holds
        if (robot.getClass() == RoboticaBot.class) {
            ((RoboticaBot) robot).wristServo.setPosition(0.2);
        }

        //Pose2d startPose = new Pose2d(12,-62, Math.toRadians(90)); // RED_BOARD
        //Pose2d startPose = new Pose2d(-36,-62, Math.toRadians(270)); // RED_AUDIENCE
        //Pose2d startPose = new Pose2d(12,62, Math.toRadians(90)); // BLUE_BOARD
        //Pose2d startPose = new Pose2d(-36,62, Math.toRadians(90)); // BLUE_AUDIENCE

        AprilTagLocalizer aprilTag = new AprilTagLocalizer(robot.getCameras());
        Pose2d startPose = null;
        while(opModeInInit() && startPose == null) {
            telemetry.log().add("Looking for AprilTag");
            startPose = estimateWithAllCameras(robot.getCameras(), aprilTag);
            telemetry.log().add("AprilTag found: " + startPose);
        }


        TensorFlowDetection.PropPosition tensorPos = null;
        if (config.tensorFlowInInit && startPose != null) {
            telemetry.log().add("Running TensorFlow in INIT: FOR DEBUGGING ONLY");

            aprilTag.close();

            tensorPos = detectProp(robot, 30);
            telemetry.log().add("Tensorflow detected: " + tensorPos);
        }

        android.util.Log.i("AUTO", "Waiting for start...");
        waitForStart();
        if (isStopRequested()) {
            android.util.Log.d("AUTO", "STOP requested after waitForStart(), returning early");
            return;
        }

        if (startPose == null) {
            telemetry.log().add("Trying to find AprilTag again");
            startPose = estimateWithAllCameras(robot.getCameras(), aprilTag);
        }

        if (startPose == null) return;
        if (robot.getClass() == RoboticaBot.class) {
            startPose = new Pose2d(startPose.getX(), startPose.getY(), startPose.getHeading() + Math.toRadians(180));
        }
        robot.getDrive().setPoseEstimate(startPose);

        if (!config.tensorFlowInInit || tensorPos == null) {
            aprilTag.close();
            tensorPos = detectProp(robot, 5);
            telemetry.log().add("Tensorflow detected: " + tensorPos);
        }

        List<Component> componentList = new ArrayList<>();

        if (config.placePixel) {
            componentList.add(new PurplePixelComponent(robot, tensorPos, Objects.equals(config.park, "inner") || Objects.equals(config.park, "board")));
        }

        switch (config.park) {
            case "outer":
                componentList.add(new ParkingOut(robot));
                break;
            case "inner":
                componentList.add(new ParkingIn(robot));
                break;
            case "board":
                componentList.add(new GoToBoard(robot, tensorPos));
                break;
        }

        for (Component component : componentList) {
            component.drive();
        }


    }




}
