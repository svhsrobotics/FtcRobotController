package org.firstinspires.ftc.teamcode.opmode.components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class ParkingOut extends Component{

    protected ParkingOut(Robot robot) {
        super(robot);
    }

    @Override
    public void drive() {getRobot().getDrive().getClass();}
    public void parkOuter() {
        //getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD

        Pose2d currentPose = getRobot().getDrive().getPoseEstimate();
        TrajectorySequence traj = null;


        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE) {
            traj = getRobot().getDrive().trajectorySequenceBuilder(currentPose)
                    .lineTo(new Vector2d(currentPose.getX(), -(4*12+9)))
                    .lineTo(new Vector2d(59, -57))
                    .build();
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD) {
            traj = getRobot().getDrive().trajectorySequenceBuilder(currentPose)
                    .lineTo(new Vector2d(currentPose.getX(), -57))
                    .lineTo(new Vector2d(59, -6))
                    .build();
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            traj = getRobot().getDrive().trajectorySequenceBuilder(currentPose)
                    .lineTo(new Vector2d(currentPose.getX(), 57))

                    .lineTo(new Vector2d(59, 57))
                    .build();


        }
    }

}

