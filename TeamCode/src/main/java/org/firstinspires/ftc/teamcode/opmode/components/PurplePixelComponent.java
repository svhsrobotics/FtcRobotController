package org.firstinspires.ftc.teamcode.opmode.components;

import static org.firstinspires.ftc.teamcode.util.Units.fi;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.RoboticaBot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;

public class PurplePixelComponent extends Component {
    Vector2d RED_AUDIENCE_TAPE_MARKS = new Vector2d(fi(-3,0), fi(-3,0));
    Vector2d BLUE_AUDIENCE_TAPE_MARKS = new Vector2d(-3 * 12, 2 * 12 + 12);
    Vector2d RED_BOARD_TAPE_MARKS = new Vector2d(fi(1,0), fi(-3,0));
    Vector2d BLUE_BOARD_TAPE_MARKS = new Vector2d(12, 2 * 12 + 12);
    private final TensorFlowDetection.PropPosition propPosition;
    private final boolean moveTowardsCenter;

    public PurplePixelComponent(Robot robot, TensorFlowDetection.PropPosition propPosition, boolean moveTowardsCenter) {
        super(robot);
        this.propPosition = propPosition;
        this.moveTowardsCenter = moveTowardsCenter;
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

        if (getRobot().getClass() == RoboticaBot.class) {
            //trajB = trajB.turn(Math.toRadians(180));
            driveBackwards = -1;
        }

        if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
            trajB.turn(Math.toRadians(90))
                    .forward(8 * driveBackwards); // orig 13
        } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
            trajB.turn(Math.toRadians(-90))
                    .forward(12 * driveBackwards);
        } else {
            if (moveTowardsCenter) {
                trajB.turn(Math.toRadians(180))
                        .forward(-16 * driveBackwards);
            } else {
                trajB.forward(10 * driveBackwards);
            }
        }

        trajB.waitSeconds(1)
                .addTemporalMarker(() -> getRobot().dropPurplePixel(true))
                .waitSeconds(2);

        //if (propPosition == TensorFlowDetection.PropPosition.CENTER && (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_AUDIENCE || getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE )) {
        if (moveTowardsCenter && propPosition == TensorFlowDetection.PropPosition.CENTER) {
            trajB.forward(-12 * driveBackwards);
        } else {
            trajB.lineTo(currentTapeMarks());
        }

        TrajectorySequence traj = trajB.build();
        getRobot().getDrive().followTrajectorySequence(traj);
        getRobot().dropPurplePixel(false);
    }
}
