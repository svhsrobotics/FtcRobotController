package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.SampleTankDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.entity.TrajectorySequenceEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).build();
//        DriveShim drive = myBot.getDrive();
//
//        Pose2d startPose = new Pose2d(-36, -56, 0);
//
//        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
//                .lineTo(new Vector2d(startPose.getX(), -3 * 12 + 2)) // drive forward to prevent the spline from cutting through the poles
//                .addDisplacementMarker(() -> {
//
//
//                    //android.util.Log.i("DROP", "Turn");
//                    System.out.println("Turn!");
//                })
//                .turn(Math.toRadians(90))
//
//                .addDisplacementMarker(() -> {
//                    //PIXEL DROP
//                    try {
//                        Thread.sleep(10000);
//                    } catch (InterruptedException e) {
//                        throw new RuntimeException(e);
//                    }
//                    System.out.println("Sleeping!");
//                    //android.util.Log.i("DROP", "SLEEPING");
//                })
//                .turn(Math.toRadians(-180))
//                .addDisplacementMarker(() -> {
//                    System.out.println("turn again");
//
//                    //android.util.Log.i("DROP", "turn again");
//                })
//                //.splineToSplineHeading(new Pose2d(3*12 + 5, -3*12, 0), 0)
//                .lineToSplineHeading(new Pose2d(2 * 12, -3 * 12 + 2, Math.toRadians(90)))
//                .lineToSplineHeading(new Pose2d(3 * 12 + 5, -3 * 12 + 2, 0))
//                .build();
//
//        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();
    }
}