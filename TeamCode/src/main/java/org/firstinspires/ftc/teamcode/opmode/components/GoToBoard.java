package org.firstinspires.ftc.teamcode.opmode.components;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;

public class GoToBoard extends Component{

    
    TensorFlowDetection.PropPosition propPos;
    public GoToBoard(Robot robot, TensorFlowDetection.PropPosition prop) {
        super(robot);
        propPos = prop;
    }

    @Override
    public void drive() {
        //getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD

        Pose2d startPose = getRobot().getDrive().getPoseEstimate();
        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(startPose);

        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE) {
            trajB = trajB.lineTo(new Vector2d(startPose.getX(), -12))
                    .lineTo(new Vector2d(46, -12));
            if (propPos == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(46, -28.25));
            } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(46, -42.5));
            } else {
                trajB = trajB.lineTo(new Vector2d(46, -35));

            }
            trajB=trajB.addTemporalMarker(() -> {
                        android.util.Log.i("PLACE PIXEL", "Placed pixel at Red Board");
                    })
                    .turnTo(0);

        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD) {
            trajB = trajB.lineTo(new Vector2d(startPose.getX(), -12))
                    .lineTo(new Vector2d(46, -12));
            if (propPos == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(46, -28.25));
            } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(46, -42.5));
            } else {
                trajB = trajB.lineTo(new Vector2d(46, -35));

            }
            trajB=trajB

                    .addTemporalMarker(() -> {
                        android.util.Log.i("PLACE PIXEL", "Placed pixel at Red Board");
                    })
                    .turnTo(0);

        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            trajB = trajB.lineTo(new Vector2d(startPose.getX(), 12))
                    .lineTo(new Vector2d(46, 12));
            if (propPos == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(46, 28.25));
            } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(46, 42.5));
            } else {
                trajB = trajB.lineTo(new Vector2d(46, 35));

            }
            trajB=trajB

            .turn(Math.toRadians(90-startPose.getHeading()))
            //TODO:fix error with temporal marker because ryan is a dum dum
            .addTemporalMarker(() -> {
                android.util.Log.i("PLACE PIXEL", "Placed pixel at Blue Board");
            })
                    .turnTo(0);


        }else {
            trajB = trajB.lineTo(new Vector2d(startPose.getX(), 12))
                    .lineTo(new Vector2d(46, 12));
            if (propPos == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(46, 28.25));
            } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(46, 42.5));
            } else {
                trajB = trajB.lineTo(new Vector2d(46, 35));
            }
            trajB=trajB
         
                    .addTemporalMarker(() -> {
                        android.util.Log.i("PLACE PIXEL", "Placed pixel at Blue Board");
                    })
                    .turnTo(0);


        }

        TrajectorySequence traj = trajB.build();
        getRobot().getDrive().followTrajectorySequence(traj);

    }


}
