package org.firstinspires.ftc.teamcode.opmode.components;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;

public class PurplePixelComponent extends Component {
    Vector2d RED_AUDIENCE_TAPE_MARKS = new Vector2d(-3 * 12, -2 * 12 - 12);
    Vector2d BLUE_AUDIENCE_TAPE_MARKS = new Vector2d(-3 * 12, 2 * 12 + 12);
    Vector2d RED_BOARD_TAPE_MARKS = new Vector2d(12, -2 * 12 - 12);
    Vector2d BLUE_BOARD_TAPE_MARKS = new Vector2d(12, 2 * 12 + 12);
    private final TensorFlowDetection.PropPosition propPosition;
    public PurplePixelComponent(Robot robot, TensorFlowDetection.PropPosition propPosition) {
        super(robot);
        this.propPosition = propPosition;
    }

    private Vector2d currentTapeMarks() {
        switch (getRobot().getDrive().currentQuadrant()) {
            case BLUE_BOARD:
                return BLUE_BOARD_TAPE_MARKS;
            case BLUE_AUDIENCE:
                return BLUE_AUDIENCE_TAPE_MARKS;
            case RED_AUDIENCE:
                return RED_AUDIENCE_TAPE_MARKS;
            case RED_BOARD:
                return RED_BOARD_TAPE_MARKS;
            default:
                return null;
        }
    }

    @Override
    public void drive() {

        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(getRobot().getDrive().getPoseEstimate());
        trajB.lineTo(currentTapeMarks());
        int driveBackwards = 1;

//        if (getRobot().getClass() == PsiBot.class) {
//            trajB = trajB.turn(Math.toRadians(180));
//            driveBackwards = -1;
//        }

        if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
            trajB.turn(Math.toRadians(90))
                    .forward(8 * driveBackwards); // orig 13
        } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
            trajB.turn(Math.toRadians(-90))
                    .forward(12 * driveBackwards);
        } else {
            trajB.turn(Math.toRadians(180))
                    .forward(-14 * driveBackwards);
        }

        trajB.addTemporalMarker(() -> getRobot().dropPurplePixel(true))
                .waitSeconds(2);

        // If this is the CENTER line, don't bother going back to the tape marks center (audience only)
        // (would end up moving the pixel)
        if (propPosition != TensorFlowDetection.PropPosition.CENTER && (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD || getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD )) {
            trajB.lineTo(currentTapeMarks());
        } else {
            // CENTER AUDIENCE special case
            // TODO: BACK UP HERE
        }

        TrajectorySequence traj = trajB.build();
        getRobot().getDrive().followTrajectorySequence(traj);
    }
}
