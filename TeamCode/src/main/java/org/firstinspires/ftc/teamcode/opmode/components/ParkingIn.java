package org.firstinspires.ftc.teamcode.opmode.components;

import static org.firstinspires.ftc.teamcode.util.Units.fi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.RoboticaBot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.opmode.TestTeleOp;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class ParkingIn extends Component{
    public ParkingIn(Robot robot) {
        super(robot);
    }

    @Override
    public void drive() {
        Pose2d currentPose = getRobot().getDrive().getPoseEstimate();
        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(currentPose);

        // If we're on the blue side, then set this to 1 foot, -1 foot on red side
        int y = (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) || (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_AUDIENCE)
                ? fi(0,10) : fi(0,-7);

        trajB.lineTo(new Vector2d(currentPose.getX(), y));
        trajB.lineToLinearHeading(new Pose2d(currentPose.getX(), y+0.1, Math.toRadians(180)));
        trajB.addTemporalMarker(() -> {
            ((RoboticaBot) getRobot()).wristLiftServo.setPosition(TestTeleOp.NEUTRAL_WRIST);
            ((RoboticaBot) getRobot()).elbowServo.setPosition(TestTeleOp.NEUTRAL_ELBOW);
        });

        trajB.lineTo(new Vector2d(fi(4,0), y));
        //trajB.turnTo(Math.toRadians(180));

        TrajectorySequence traj = trajB.build();
        getRobot().getDrive().followTrajectorySequence(traj);

    }

}
