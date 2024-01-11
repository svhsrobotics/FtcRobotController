package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class ParkingOut extends Component{

    protected ParkingOut(Robot robot) {
        super(robot);
    }

    @Override
    public void drive() {

        //getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD

        Pose2d currentPose = getRobot().getDrive().getPoseEstimate();
        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(currentPose);



        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE) {
            trajB =trajB.lineTo(new Vector2d(currentPose.getX(), -(4*12+9)))
                    .lineTo(new Vector2d(59, -57));
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD) {
            trajB = trajB.lineTo(new Vector2d(currentPose.getX(), -57))
                    .lineTo(new Vector2d(59, -57));

        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            trajB = trajB.lineTo(new Vector2d(currentPose.getX(), 57))
                    .lineTo(new Vector2d(59, 57));
        } else {
            trajB = trajB.lineTo(new Vector2d(currentPose.getX(), 57))
                    .lineTo(new Vector2d(59, 57));
        }
        TrajectorySequence traj = trajB.build();
        getRobot().getDrive().followTrajectorySequence(traj);
    }
//ryan i had to fix ur code it didn't copy ParkingIn correctly - not the values just formatting
}

