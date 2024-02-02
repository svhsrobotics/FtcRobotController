package org.firstinspires.ftc.teamcode.opmode.components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;

public class PurplePixelComponent extends Component {
    Pose2d RED_AUDIENCE_TAPE_MARKS = new Pose2d(-3 * 12, -2 * 12 - 12);
    Pose2d BLUE_AUDIENCE_TAPE_MARKS = new Pose2d(-3 * 12, 2 * 12 + 12);
    Pose2d RED_BOARD_TAPE_MARKS = new Pose2d(12, -2 * 12 - 12);
    Pose2d BLUE_BOARD_TAPE_MARKS = new Pose2d(12, 2 * 12 + 12);
    private final TensorFlowDetection.PropPosition propPosition;
    public PurplePixelComponent(Robot robot, TensorFlowDetection.PropPosition propPosition) {
        super(robot);
        this.propPosition = propPosition;
    }

    private TrajectorySequenceBuilder goToTapeCenter(TrajectorySequenceBuilder trajB) {
        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            trajB = trajB.lineTo(new Vector2d(BLUE_BOARD_TAPE_MARKS.getX(), BLUE_BOARD_TAPE_MARKS.getY()));
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_AUDIENCE) {
            trajB = trajB.lineTo(new Vector2d(BLUE_AUDIENCE_TAPE_MARKS.getX(), BLUE_AUDIENCE_TAPE_MARKS.getY()));
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE) {
            trajB = trajB.lineTo(new Vector2d(RED_AUDIENCE_TAPE_MARKS.getX(), RED_AUDIENCE_TAPE_MARKS.getY()));
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD) {
            trajB = trajB.lineTo(new Vector2d(RED_BOARD_TAPE_MARKS.getX(), RED_BOARD_TAPE_MARKS.getY()));
        }
        return trajB;
    }

    @Override
    public void drive() {

        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(getRobot().getDrive().getPoseEstimate());
        trajB = goToTapeCenter(trajB);
        int driveBackwards = 1;

//        if (getRobot().getClass() == PsiBot.class) {
//            trajB = trajB.turn(Math.toRadians(180));
//            driveBackwards = -1;
//        }

        if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
            trajB = trajB.turn(Math.toRadians(90))
                    .forward(8 * driveBackwards); // orig 13
        } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
            trajB = trajB.turn(Math.toRadians(-90))
                    .forward(12 * driveBackwards);
        } else {
            trajB = trajB.turn(Math.toRadians(180))
                    .forward(-8 * driveBackwards);
        }

        trajB = trajB.addTemporalMarker(() -> getRobot().dropPurplePixel(true))
                .waitSeconds(2);
        trajB = goToTapeCenter(trajB);


        TrajectorySequence traj = trajB.build();
        getRobot().getDrive().followTrajectorySequence(traj);
    }
}
