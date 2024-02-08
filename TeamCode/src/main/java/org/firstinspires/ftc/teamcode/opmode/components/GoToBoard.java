package org.firstinspires.ftc.teamcode.opmode.components;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.PsiBot;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;
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
        //getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD

        Pose2d startPose = getRobot().getDrive().getPoseEstimate();
        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(startPose);

        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE) {
            trajB = trajB.lineTo(new Vector2d(startPose.getX(), -12))
                    .lineTo(new Vector2d(40, -12));
            if (propPos == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(40, -28.25));
            } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(40, -42.5));
            } else {
                trajB = trajB.lineTo(new Vector2d(40, -35));

            }
            trajB = trajB.addTemporalMarker(() -> {
                        android.util.Log.i("PLACE PIXEL", "Placed pixel at Red Board");
                    })
                    .turnTo(0);

        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD) {
            trajB = trajB.lineTo(new Vector2d(startPose.getX(), -12))
                    .lineTo(new Vector2d(40, -12));
            if (propPos == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(40, -28.25));
            } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(40, -42.5));
            } else {
                trajB = trajB.lineTo(new Vector2d(40, -35));

            }
            trajB = trajB

                    .addTemporalMarker(() -> {
                        android.util.Log.i("PLACE PIXEL", "Placed pixel at Red Board");
                    })
                    .turnTo(0);

        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
            trajB = trajB.lineTo(new Vector2d(startPose.getX(), 12))
                    .lineTo(new Vector2d(40, 12));
            if (propPos == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(40, 28.25));
            } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(40, 42.5));
            } else {
                trajB = trajB.lineTo(new Vector2d(40, 35));

            }
            trajB = trajB

                    .turn(Math.toRadians(90 - startPose.getHeading()))
                    //TODO:fix error with temporal marker because ryan is a dum dum
                    .addTemporalMarker(() -> {
                        android.util.Log.i("PLACE PIXEL", "Placed pixel at Blue Board");
                    })
                    .turnTo(0);


        } else {
            trajB = trajB.lineTo(new Vector2d(startPose.getX(), 12))
                    .lineTo(new Vector2d(40, 12));
            if (propPos == TensorFlowDetection.PropPosition.LEFT) {
                trajB = trajB.lineTo(new Vector2d(40, 28.25));
            } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
                trajB = trajB.lineTo(new Vector2d(40, 42.5));
            } else {
                trajB = trajB.lineTo(new Vector2d(40, 38.5));
            }
            trajB = trajB

                    .addTemporalMarker(() -> {
                        android.util.Log.i("PLACE PIXEL", "Placed pixel at Blue Board");
                    })
                    .turnTo(0);


        }
        if (getRobot().getClass() == PsiBot.class) {
            trajB.turn(Math.toRadians(180));
        }

        TrajectorySequence traj = trajB.build();
        Log.i("PROGRESS", "1");
        GlobalOpMode.opMode.telemetry.log().add("1");
        getRobot().getDrive().followTrajectorySequence(traj);
        Log.i("PROGRESS", "2");
        GlobalOpMode.opMode.telemetry.log().add("2");
        while ((((PsiBot) getRobot()).armMotor.getCurrentPosition() >= -1000 || ((PsiBot) getRobot()).armMotor.getCurrentPosition() <= -1100))
            if (getRobot().getClass() == PsiBot.class) {

                ((PsiBot) getRobot()).armMotor.setTargetPosition(-1052);
                ((PsiBot) getRobot()).armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((PsiBot) getRobot()).armMotor.setPower(.1);
                Log.i("PROGRESS", "4");

            }
        Log.i("PROGRESS", "3");

    }



}
