package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TestBot;
import org.firstinspires.ftc.teamcode.opmode.components.Component;
import org.firstinspires.ftc.teamcode.opmode.components.GoToBoard;
import org.firstinspires.ftc.teamcode.opmode.components.ParkingIn;
import org.firstinspires.ftc.teamcode.opmode.components.ParkingOut;
import org.firstinspires.ftc.teamcode.opmode.components.PurplePixelComponent;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@Autonomous
public class TestAuto extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        TestBot testBot = new TestBot(hardwareMap);

        //Pose2d startPose = new Pose2d(12,-62, Math.toRadians(270)); // RED_BOARD
        Pose2d startPose = new Pose2d(-36,-62, Math.toRadians(270)); // RED_AUDIENCE
        //Pose2d startPose = new Pose2d(12,62, Math.toRadians(90)); // BLUE_BOARD
        //Pose2d startPose = new Pose2d(-36,62, Math.toRadians(90)); // BLUE_AUDIENCE

        testBot.getDrive().setPoseEstimate(startPose);

        List<Component> componentList = new ArrayList<>();
        Configuration config = Configurator.load();
        if (config.placePixel) {
            componentList.add(new PurplePixelComponent(testBot, TensorFlowDetection.PropPosition.RIGHT));
        }
        if (Objects.equals(config.park, "outer")) {
            componentList.add(new ParkingOut(testBot));
        } else if (Objects.equals(config.park, "inner")) {
            componentList.add(new ParkingIn(testBot));
        } else  if (Objects.equals(config.park, "board")) {
            componentList.add(new GoToBoard(testBot, TensorFlowDetection.PropPosition.RIGHT));
        }

        waitForStart();

        for (Component component : componentList) {
            component.drive();
        }

    }




}
