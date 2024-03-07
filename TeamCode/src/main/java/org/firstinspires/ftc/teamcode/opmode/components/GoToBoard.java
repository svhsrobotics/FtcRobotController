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
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
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
        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(startPose);

        // If we're on the blue side, then set this to 1 foot, -1 foot on red side
        int y = (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) || (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_AUDIENCE) ? fi(3,0) : fi(-3,0);
        int x = fi(3, 4);
        // Drive in front of board, a little back so we have room to raise the arm
        //trajB.lineTo(new Vector2d(x, y));
        trajB.lineToLinearHeading(new Pose2d(x,y,Math.toRadians(180)));
        //trajB.lineTo(new Vector2d(x, y-1));
        //trajB.turnTo(Math.toRadians(180));

        getRobot().getDrive().followTrajectorySequence(trajB.build());

        // TODO: Raise arm here (sleeping to simulate)
        GlobalOpMode.opMode.sleep(400);

        // Drive right up to the board
        TrajectorySequenceBuilder trajC = getRobot().getDrive().trajectorySequenceBuilder(getRobot().getDrive().getPoseEstimate());
        // depends on which tensor pos
        if (propPos == TensorFlowDetection.PropPosition.CENTER) {
            trajC.lineTo(new Vector2d(x+8, y));
        } else if (propPos == TensorFlowDetection.PropPosition.LEFT) {
            trajC.lineTo(new Vector2d(x+8, y - 6));
        } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
            trajC.lineTo(new Vector2d(x+8, y + 6));
        }
        getRobot().getDrive().followTrajectorySequence(trajC.build());

        // TODO: Drop pixel here (sleeping to simulate)
        GlobalOpMode.opMode.sleep(400);

        // Back up to give room for the arm to come down
        TrajectorySequenceBuilder trajE = getRobot().getDrive().trajectorySequenceBuilder(getRobot().getDrive().getPoseEstimate());
        trajE.forward(8);
        getRobot().getDrive().followTrajectorySequence(trajE.build());

        // TODO: Bring arm down
    }



}
