package org.firstinspires.ftc.teamcode.opmode.components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class ParkingOut extends Component{

    public ParkingOut(Robot robot) {
        super(robot);
    }

    @Override
    public void drive() {

        //getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD

        Pose2d currentPose = getRobot().getDrive().getPoseEstimate();
        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(currentPose);



        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE) {
            trajB =trajB.lineTo(new Vector2d(currentPose.getX(), -(5*12 + 1)))
                    .lineTo(new Vector2d(55, -61));
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD) {
            trajB = trajB.lineTo(new Vector2d(currentPose.getX(), -61))
                    .lineTo(new Vector2d(55, -61));

        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            trajB = trajB.lineTo(new Vector2d(currentPose.getX(), 59))
                    .lineTo(new Vector2d(55, 59));
        } else {
        trajB = trajB.lineTo(new Vector2d(currentPose.getX(), 59))
                .lineTo(new Vector2d(55, 59));
    }
        TrajectorySequence traj = trajB.build();
        getRobot().getDrive().followTrajectorySequence(traj);
    }

}

