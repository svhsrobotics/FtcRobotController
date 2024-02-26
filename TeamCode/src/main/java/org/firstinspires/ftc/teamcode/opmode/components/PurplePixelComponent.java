package org.firstinspires.ftc.teamcode.opmode.components;

import static org.firstinspires.ftc.teamcode.util.Units.fi;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.PsiBot;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.RoboticaBot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;

public class PurplePixelComponent extends Component {
    Vector2d RED_AUDIENCE_TAPE_MARKS = new Vector2d(fi(-3,0), fi(-2,-10));
    Vector2d BLUE_AUDIENCE_TAPE_MARKS = new Vector2d(-3 * 12, 2 * 12 + 12);
    Vector2d RED_BOARD_TAPE_MARKS = new Vector2d(14.5, (-2*12)-14);
    Vector2d BLUE_BOARD_TAPE_MARKS = new Vector2d(13.5, 2 * 12 + 14);
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
        double PURPLE_X_OFFSET=0;
        double PURPLE_Y_OFFSET=0;

        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(getRobot().getDrive().getPoseEstimate());
        trajB.lineTo(currentTapeMarks());
        int driveBackwards = 1;

        if (getRobot().getClass() == RoboticaBot.class) {
            //trajB = trajB.turn(Math.toRadians(180));
            driveBackwards = -1;
        } else {
             PURPLE_X_OFFSET=0;
             PURPLE_Y_OFFSET=0;
        }

        if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
            trajB.turn(Math.toRadians(90))
                    .forward(10 * driveBackwards)
                    .forward(-3.5); // orig 13
            trajB
                    .addTemporalMarker(() -> getRobot().dropPurplePixel(true))
                    .waitSeconds(2)
            .addTemporalMarker(() -> getRobot().dropPurplePixel(false));
        } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
            trajB.turn(Math.toRadians(-90))
                    .forward(8 * driveBackwards)
                            .forward(-2.5);


            trajB
                    .addTemporalMarker(() -> getRobot().dropPurplePixel(true))
                    .waitSeconds(2);
        } else {
            if (moveTowardsCenter) {

                trajB.forward(4)
                        .turn(Math.toRadians(180))
                        .forward(-20 * driveBackwards)
                        .forward(8 * driveBackwards + PURPLE_Y_OFFSET);
                trajB
                        .addTemporalMarker(() -> getRobot().dropPurplePixel(true))
                        .waitSeconds(1);

            } else {
                trajB.forward(10 * driveBackwards)
                        .addTemporalMarker(() -> getRobot().dropPurplePixel(true))
                        .waitSeconds(1);
            }
        }



        //if (propPosition == TensorFlowDetection.PropPosition.CENTER && (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_AUDIENCE || getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE )) {
        if (moveTowardsCenter && propPosition == TensorFlowDetection.PropPosition.CENTER) {
            trajB.forward(-12 * driveBackwards);
        } else {
            trajB.lineTo(currentTapeMarks());
        }

        TrajectorySequence traj = trajB.build();
        getRobot().getDrive().followTrajectorySequence(traj);
        if (getRobot().getClass() == PsiBot.class) {
            while (((PsiBot) getRobot()).armMotor.getCurrentPosition()<-10 || ((PsiBot) getRobot()).armMotor.getCurrentPosition()>10) {
                ((PsiBot) getRobot()).armMotor.setTargetPosition(0);
                ((PsiBot) getRobot()).armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((PsiBot) getRobot()).armMotor.setPower(.1);
            }

        }
        getRobot().dropPurplePixel(false);

    }
}
