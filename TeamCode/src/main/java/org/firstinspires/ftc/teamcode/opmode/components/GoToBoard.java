package org.firstinspires.ftc.teamcode.opmode.components;

import static org.firstinspires.ftc.teamcode.util.Units.fi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.RoboticaBot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
import org.firstinspires.ftc.teamcode.opmode.TestTeleOp;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;

public class GoToBoard extends Component {
    TensorFlowDetection.PropPosition propPos;

    public GoToBoard(Robot robot, TensorFlowDetection.PropPosition prop) {
        super(robot);
        propPos = prop;
    }

    @Override
    public void drive() {
        Pose2d startPose = getRobot().getDrive().getPoseEstimate();
        TrajectorySequenceBuilder traj = getRobot().getDrive().trajectorySequenceBuilder(startPose);

        // If we're on the blue side, then set this to 1 foot, -1 foot on red side
        int board_y = (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) || (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_AUDIENCE)
                ? fi(2,12) : fi(-2,9);
        int board_x = fi(3, 4);

        // Drive in front of board, a little back so we have room to raise the arm
        traj.lineToLinearHeading(new Pose2d(board_x,board_y,Math.toRadians(180)));

        traj.waitSeconds(0.4); // Let the bot stop moving so that it doesn't mess up when recalibrating
        traj.addTemporalMarker(() -> {
            // TODO: Raise arm here (sleeping to simulate)
            RoboticaBot rrobot = (RoboticaBot) getRobot();
            rrobot.wristLiftServo.setPosition(TestTeleOp.RAISED_WRIST);
            rrobot.elbowServo.setPosition(TestTeleOp.RAISED_ELBOW);
//            rrobot.recalibrateShoulder();
//
//            rrobot.setShoulderTargetPosition(TestTeleOp.RAISED_ARM);

        });
        //traj.waitSeconds(1); // Wait for the elbow
        traj.addTemporalMarker(() -> {
            RoboticaBot rrobot = (RoboticaBot) getRobot();

            rrobot.recalibrateShoulder();
            rrobot.setShoulderTargetPosition(TestTeleOp.RAISED_ARM);
        });
        traj.waitSeconds(1);

        // Drive right up to the board
        // depends on which tensor pos
        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            board_x += 2;
        }
        if (propPos == TensorFlowDetection.PropPosition.CENTER) {
            traj.lineTo(new Vector2d(board_x+8, board_y));
        } else if (propPos == TensorFlowDetection.PropPosition.LEFT) {
            //if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_AUDIENCE)
            //    traj.lineTo(new Vector2d(board_x+8, board_y + 8));
            //else
            traj.lineTo(new Vector2d(board_x+8, board_y + 8));
        } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
            //if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD)
            //    traj.lineTo(new Vector2d(board_x+8, board_y - 8));
            //else
            traj.lineTo(new Vector2d(board_x+8, board_y - 6));
        }

        traj.addTemporalMarker(() -> {
            // TODO: Drop pixel here (sleeping to simulate)
            RoboticaBot rrobot = (RoboticaBot) getRobot();
            rrobot.pinchServo.setPosition(1.0);
        });
        traj.waitSeconds(0.5); // Wait for the arm


        // Back up to give room for the arm to come down
        //traj.forward(8);
        getRobot().getDrive().followTrajectorySequence(traj.build());

        // TODO: Bring arm down
        ((RoboticaBot) getRobot()).setShoulderTargetPosition(TestTeleOp.NEUTRAL_ARM);
    }



}
