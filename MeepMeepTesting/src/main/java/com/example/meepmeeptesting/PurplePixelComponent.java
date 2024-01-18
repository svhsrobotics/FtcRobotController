package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;


public class PurplePixelComponent extends Component {
    private TensorFlowDetection.PropPosition propPosition;
    public PurplePixelComponent(Robot robot, TensorFlowDetection.PropPosition propPosition) {
        super(robot);
        this.propPosition = propPosition;
    }

    @Override
    public void drive() {
        //Pose2d getRobot().getDrive().getPoseEstimate() = getRobot().getDrive().getPoseEstimate();
        Pose2d RED_AUDIENCE_TAPE_MARKS = new Pose2d(-3*12, -2*12 - 8);
        Pose2d BLUE_AUDIENCE_TAPE_MARKS = new Pose2d(-3*12, 2*12 + 8);
        Pose2d RED_BOARD_TAPE_MARKS = new Pose2d(12, -2*12 - 8);
        Pose2d BLUE_BOARD_TAPE_MARKS = new Pose2d(12, 2*12 + 8);

        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(getRobot().getDrive().getPoseEstimate());



        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(BLUE_BOARD_TAPE_MARKS.getX(), BLUE_BOARD_TAPE_MARKS.getY()))
                        .strafeLeft(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });

            } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(BLUE_BOARD_TAPE_MARKS.getX(), BLUE_BOARD_TAPE_MARKS.getY()))
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


            }
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_AUDIENCE) {
            if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(BLUE_AUDIENCE_TAPE_MARKS.getX(), BLUE_AUDIENCE_TAPE_MARKS.getY()))
                        .strafeLeft(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });

            } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(BLUE_AUDIENCE_TAPE_MARKS.getX(), BLUE_AUDIENCE_TAPE_MARKS.getY()))
                        .strafeRight(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });

            } else {
                trajB = trajB.lineTo(new Vector2d(BLUE_AUDIENCE_TAPE_MARKS.getX(), BLUE_AUDIENCE_TAPE_MARKS.getY()))
                        .forward(-8)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });


            }
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE) {
            if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(RED_AUDIENCE_TAPE_MARKS.getX(), RED_AUDIENCE_TAPE_MARKS.getY()))
                        .strafeLeft(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });

            } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(RED_AUDIENCE_TAPE_MARKS.getX(), RED_AUDIENCE_TAPE_MARKS.getY()))
                        .strafeRight(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });

            } else {
                trajB = trajB.lineTo(new Vector2d(RED_AUDIENCE_TAPE_MARKS.getX(), RED_AUDIENCE_TAPE_MARKS.getY()))
                        .forward(-8)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });


            }
        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD) {
            if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(RED_BOARD_TAPE_MARKS.getX(), RED_BOARD_TAPE_MARKS.getY()))
                        .strafeLeft(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });

            } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(RED_BOARD_TAPE_MARKS.getX(), RED_BOARD_TAPE_MARKS.getY()))
                        .strafeRight(-12)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });

            } else {
                trajB = trajB.lineTo(new Vector2d(RED_BOARD_TAPE_MARKS.getX(), RED_BOARD_TAPE_MARKS.getY()))
                        .forward(-8)
                        .addTemporalMarker(() -> {
                            getRobot().dropPurplePixel(true);
                        });


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
