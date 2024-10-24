package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class GoToBoard extends Component{

    protected GoToBoard(Robot robot) {
        super(robot);
    }

    @Override
    public void drive() {
        //getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD
        
        Pose2d startPose = getRobot().getDrive().getPoseEstimate();
     
        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(startPose);

        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE) {
            trajB = trajB.lineTo(new Vector2d(startPose.getX(), -36))
                    .lineTo(new Vector2d(50, -36))

            .turn(Math.toRadians(-90-startPose.getHeading()))
            //TODO:fix error with temporal marker because ryan is a dum dum
           .addTemporalMarker(() -> {
    //            android.util.Log.i("PLACE PIXEL", "Placed pixel at Red Board");
            });

        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD) {
            trajB = trajB.lineTo(new Vector2d(startPose.getX(), -36))
                    .lineTo(new Vector2d(50, -36))

                   .turn(Math.toRadians(-90-startPose.getHeading()))

            //TODO:fix error with temporal marker because ryan is a dum dum
            .addTemporalMarker(() -> {
    //            android.util.Log.i("PLACE PIXEL", "Placed pixel at Red Board");
            });

        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            trajB = trajB.lineTo(new Vector2d(startPose.getX(), 36))
                    .lineTo(new Vector2d(50, 36))

            .turn(Math.toRadians(90-startPose.getHeading()))

            //TODO:fix error with temporal marker because ryan is a dum dum
            .addTemporalMarker(() -> {
     //           android.util.Log.i("PLACE PIXEL", "Placed pixel at Blue Board");
            });

        }else {
            trajB = trajB.lineTo(new Vector2d(startPose.getX(), 36))
                    .lineTo(new Vector2d(50, 36))

            .turn(Math.toRadians(90-startPose.getHeading()))


            //TODO:fix error with temporal marker because ryan is a dum dum
            .addTemporalMarker(() -> {
     //           android.util.Log.i("PLACE PIXEL", "Placed pixel at Blue Board");
            });
                    //.turn

        }

        TrajectorySequence traj = trajB.build();
        getRobot().getDrive().followTrajectorySequence(traj);

    }




}
