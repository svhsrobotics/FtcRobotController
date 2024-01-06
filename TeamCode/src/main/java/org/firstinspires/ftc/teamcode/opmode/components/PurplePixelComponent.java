package org.firstinspires.ftc.teamcode.opmode.components;

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
        TrajectorySequenceBuilder trajBuilder = getRobot().getDrive().trajectorySequenceBuilder(getRobot().getDrive().getPoseEstimate());

        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            if (propPosition == TensorFlowDetection.PropPosition.LEFT) {
                trajBuilder = trajBuilder.lineTo(new Vector2d(3 * 12 + 7, 5 * 12 + 4));
            } else if (propPosition == TensorFlowDetection.PropPosition.RIGHT) {
                trajBuilder = trajBuilder.lineTo(new Vector2d(3 * 12 + 7, -5 * 12 + 4));
            } else {
                // CENTER, drive straight forward
                trajBuilder = trajBuilder.lineTo(BLUE_BOARD_CENTER_LINE);
            }
        }

        TrajectorySequence traj = trajBuilder.build();
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
