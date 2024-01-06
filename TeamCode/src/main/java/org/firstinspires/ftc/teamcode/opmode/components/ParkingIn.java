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
        getRobot().getDrive().getClass();
    }

    public void parkInner() {
        //getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD

        Pose2d currentPose = getRobot().getDrive().getPoseEstimate();
        TrajectorySequence traj = null;

        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE) {
            traj = getRobot().getDrive().trajectorySequenceBuilder(currentPose)
                    .lineTo(new Vector2d(currentPose.getX(), -6))
                    .lineTo(new Vector2d(currentPose.getX(), -5 * 12 + 3))
                    .turn(Math.toRadians(90))
                    .lineTo(new Vector2d(3 * 12 + 7, -6))
                    .build();
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD) {
            traj = getRobot().getDrive().trajectorySequenceBuilder(currentPose)
                    .lineTo(new Vector2d(currentPose.getX(), -6))
                    .lineTo(new Vector2d(currentPose.getX(), -5 * 12 + 3))
                    .turn(Math.toRadians(90))
                    .lineTo(new Vector2d(3 * 12 + 7, -6))
                    .build();
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            traj = getRobot().getDrive().trajectorySequenceBuilder(currentPose)
                    .lineTo(new Vector2d(currentPose.getX(), 6))
                    .lineTo(new Vector2d(currentPose.getX(), 5 * 12 + 3))
                    .turn(Math.toRadians(90))
                    .lineTo(new Vector2d(3 * 12 + 7, 6))
                    .build();

        }

    }


}
