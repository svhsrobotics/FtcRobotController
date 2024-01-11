package org.firstinspires.ftc.teamcode.opmode.components;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class ParkingIn extends Component{


    protected ParkingIn(Robot robot) {
        super(robot);
    }

    @Override
    public void drive() {
        //getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD

        Pose2d currentPose = getRobot().getDrive().getPoseEstimate();
        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(currentPose);

        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE) {
            trajB = trajB.lineTo(new Vector2d(currentPose.getX(), -12))
                    .lineTo(new Vector2d(6 * 12 + 14, -12));
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD) {
            trajB = trajB.lineTo(new Vector2d(currentPose.getX(), -12))
                    .lineTo(new Vector2d(6 * 12 + 14, -12));
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            trajB = trajB.lineTo(new Vector2d(currentPose.getX(), 12))
                    .lineTo(new Vector2d(6 * 12 + 14, 12));
        } else {
            trajB = trajB.lineTo(new Vector2d(currentPose.getX(), 12))
                    .lineTo(new Vector2d(6 * 12 + 14, 12));
        }

        TrajectorySequence traj = trajB.build();
        getRobot().getDrive().followTrajectorySequence(traj);

    }

}
