package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//import org.firstinspires.ftc.teamcode.opmode.AutoPaths;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        //Pose2d startPose = new Pose2d(12,-62, Math.toRadians(270)); // RED_BOARD
        //Pose2d startPose = new Pose2d(-36,-62, Math.toRadians(270)); // RED_AUDIENCE
        //Pose2d startPose = new Pose2d(12,62, Math.toRadians(90)); // BLUE_BOARD
        Pose2d startPose = new Pose2d(-36,62, Math.toRadians(90)); // BLUE_AUDIENCE

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> {
                    Robot robot = new Robot(drive);
                    robot.getDrive().setPoseEstimate(startPose);

                    // CHANGE THESE LINES TO RUN ANOTHER COMPONENT
//                    SampleComponent sampleComponent = new SampleComponent(robot);
//                    sampleComponent.drive();
                    PurplePixelComponent purplePixelComponent = new PurplePixelComponent(robot, TensorFlowDetection.PropPosition.CENTER);
                    purplePixelComponent.drive();
                    // END CHANGE LINES

                    return robot.getCurrentTrajectorySequence(); // This is a dirty hack, and assumes the component only calls followTrajectorySequence once
                });


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}