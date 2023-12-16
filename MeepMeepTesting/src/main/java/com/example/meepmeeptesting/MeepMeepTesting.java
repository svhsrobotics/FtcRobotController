package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(12,62, Math.toRadians(270));

        Vector2d BLUE_BOARD_CENTER_LINE = new Vector2d(12, 24.5);
        Vector2d BOT_DROPPER_OFFSET = new Vector2d(4,4.5);
        Vector2d BLUE_PARK = new Vector2d(48, 48);
        Vector2d BLUE_BOARD_LEFT_LINE = new Vector2d(23, 30);
        //double CENTER_BOARD_OFFSET_X = 12;
        //double CENTER_BOARD_OFFSET_Y
        //double BOT_DROPPER_OFFSET_X = 4; // Subtract from X coord?
        //double BOT_DROPPER_OFFSET_Y = 4.5;


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(BLUE_BOARD_LEFT_LINE.getX() - BOT_DROPPER_OFFSET.getY(), BLUE_BOARD_LEFT_LINE.getY() + BOT_DROPPER_OFFSET.getY()))
                                // Drop the pixel
                                .addTemporalMarker(()->{
                                    //PIXEL DROP
                                    //Log.i("DROP", "dropping purple");
                                    //purpleServo.setPosition(1);
                                })
                                .waitSeconds(1.0)
                                // Back up to the BLUE PARK Y coord
                                .lineTo(new Vector2d(BLUE_BOARD_LEFT_LINE.getX(), BLUE_PARK.getY()))
                                // Face forwards
                                .turn(Math.toRadians(90))
                                // Park
                                .lineTo(BLUE_PARK)
                                .build()
                        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}