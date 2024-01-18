package org.firstinspires.ftc.teamcode.opmode.components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;

public class PurplePixelComponent extends Component {
    private TensorFlowDetection.PropPosition propPosition;
    public PurplePixelComponent(Robot robot, TensorFlowDetection.PropPosition propPosition) {
        super(robot);
        this.propPosition = propPosition;
    }

    @Override
    public void drive() {
        //Pose2d getRobot().getDrive().getPoseEstimate() = getRobot().getDrive().getPoseEstimate();
        Pose2d RED_AUDIENCE_TAPE_MARKS = new Pose2d(-3*12, -2*12 - 12);
        Pose2d BLUE_AUDIENCE_TAPE_MARKS = new Pose2d(-3*12, 2*12 + 12);
        Pose2d RED_BOARD_TAPE_MARKS = new Pose2d(12, -2*12 - 12);
        Pose2d BLUE_BOARD_TAPE_MARKS = new Pose2d(12, 2*12 + 12);

        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(getRobot().getDrive().getPoseEstimate());

        trajB = trajB.lineTo(new Vector2d(BLUE_BOARD_TAPE_MARKS.getX(), BLUE_BOARD_TAPE_MARKS.getY()))



        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
                        .turn(90)
                        .strafeLeft(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });

            } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(BLUE_BOARD_TAPE_MARKS.getX(), BLUE_BOARD_TAPE_MARKS.getY()))
                        .turn(90)
                        .strafeRight(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });

            } else {
                trajB = trajB.lineTo(new Vector2d(BLUE_BOARD_TAPE_MARKS.getX(), BLUE_BOARD_TAPE_MARKS.getY()))
                        .forward(-8)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });
//                        .turn(90);


            }
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_AUDIENCE) {
            if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(BLUE_AUDIENCE_TAPE_MARKS.getX(), BLUE_AUDIENCE_TAPE_MARKS.getY()))
                        .strafeLeft(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });
//                        .turn(90);

            } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(BLUE_AUDIENCE_TAPE_MARKS.getX(), BLUE_AUDIENCE_TAPE_MARKS.getY()))

                        .strafeRight(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });
//                        .turn(90);

            } else {
                trajB = trajB.lineTo(new Vector2d(BLUE_AUDIENCE_TAPE_MARKS.getX(), BLUE_AUDIENCE_TAPE_MARKS.getY()))
                        .forward(-8)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });
//                        .turn(90);


            }
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE) {
            if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(RED_AUDIENCE_TAPE_MARKS.getX(), RED_AUDIENCE_TAPE_MARKS.getY()))
                        .strafeLeft(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });
//                        .turn(-90);

            } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(RED_AUDIENCE_TAPE_MARKS.getX(), RED_AUDIENCE_TAPE_MARKS.getY()))
                        .strafeRight(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });
//                        .turn(-90);

            } else {
                trajB = trajB.lineTo(new Vector2d(RED_AUDIENCE_TAPE_MARKS.getX(), RED_AUDIENCE_TAPE_MARKS.getY()))
                        .forward(-8)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        })
//                        .turn(-90);


            }
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD) {
            if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(RED_BOARD_TAPE_MARKS.getX(), RED_BOARD_TAPE_MARKS.getY()))
                        .strafeLeft(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });
//                        .turn(-90);

            } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(RED_BOARD_TAPE_MARKS.getX(), RED_BOARD_TAPE_MARKS.getY()))
                        .strafeRight(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });
//                        .turn(-90);

            } else {
                trajB = trajB.lineTo(new Vector2d(RED_BOARD_TAPE_MARKS.getX(), RED_BOARD_TAPE_MARKS.getY()))
                        .forward(-8)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });
//                        .turn(-90);


            }
        }



        TrajectorySequence traj = trajB.build();
        getRobot().getDrive().followTrajectorySequence(traj);


//        trajBuilder = trajBuilder.lineTo(new Vector2d(startPose.getX(), -2*12 - 3));
//                .addTemporalMarker(() -> {
//                    Log.i("DROP", "dropping purple");
//                    purpleServo.setPosition(1);
//                })
//                .waitSeconds(1)
//                .lineTo(new Vector2d(startPose.getX(), -5 * 12+3))
//                .turn(Math.toRadians(90))
//                .lineTo(new Vector2d(3*12+7, -5*12+4))
//                .build();

    }
}
