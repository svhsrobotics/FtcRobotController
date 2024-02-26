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
        int y = (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) || (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_AUDIENCE) ? fi(4,0) : fi(-4,0);

        trajB.lineTo(new Vector2d(startPose.getX(), y))
            .lineTo(new Vector2d(fi(3, 10), y))
                .turnTo(Math.toRadians(0))
            //.turn(Math.toRadians(90))
            .lineTo(new Vector2d(fi(3,8), fi(-2,11)));

        TrajectorySequence traj = trajB.build();

        getRobot().getDrive().followTrajectorySequence(traj);

        if (getRobot().getClass() == RoboticaBot.class) {
            ((RoboticaBot) getRobot()).recalibrateShoulder();
            ((RoboticaBot) getRobot()).setShoulderTargetPosition(TestTeleOp.RAISED_ARM);
        }

        //GlobalOpMode.opMode.sleep(2000);

        TrajectorySequenceBuilder trajC = getRobot().getDrive().trajectorySequenceBuilder(getRobot().getDrive().getPoseEstimate());
        trajC.back(5);
        getRobot().getDrive().followTrajectorySequence(trajC.build());

        ((RoboticaBot) getRobot()).elbowServo.innerServo.setPower(-1.0);
        GlobalOpMode.opMode.sleep(400);
        ((RoboticaBot) getRobot()).elbowServo.innerServo.setPower(0.0);
        ((RoboticaBot) getRobot()).wristTwistServo.setPosition(0.5);
        ((RoboticaBot) getRobot()).wristLiftServo.setPosition(TestTeleOp.RAISED_WRIST);

        TrajectorySequenceBuilder trajD = getRobot().getDrive().trajectorySequenceBuilder(getRobot().getDrive().getPoseEstimate());
        trajD.forward(8);
        getRobot().getDrive().followTrajectorySequence(trajD.build());

        ((RoboticaBot) getRobot()).pinchServo.innerServo.setPower(1.0); // OPEN

        GlobalOpMode.opMode.sleep(900);

        ((RoboticaBot) getRobot()).pinchServo.innerServo.setPower(0.0);

        TrajectorySequenceBuilder trajE = getRobot().getDrive().trajectorySequenceBuilder(getRobot().getDrive().getPoseEstimate());
        trajE.back(8);
        getRobot().getDrive().followTrajectorySequence(trajE.build());


        GlobalOpMode.opMode.sleep(30000);






//        //getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD
//
//        Pose2d startPose = getRobot().getDrive().getPoseEstimate();
//        TrajectorySequenceBuilder trajB = getRobot().getDrive().trajectorySequenceBuilder(startPose);
//
//        if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_AUDIENCE) {
//            trajB = trajB.lineTo(new Vector2d(startPose.getX(), -12))
//                    .lineTo(new Vector2d(40, -12));
//            if (propPos == TensorFlowDetection.PropPosition.LEFT) {
//                trajB = trajB.lineTo(new Vector2d(40, -28.25));
//            } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
//                trajB = trajB.lineTo(new Vector2d(40, -42.5));
//            } else {
//                trajB = trajB.lineTo(new Vector2d(40, -35));
//
//            }
//            trajB = trajB.addTemporalMarker(() -> {
//                        android.util.Log.i("PLACE PIXEL", "Placed pixel at Red Board");
//                    })
//                    .turnTo(0);
//
//        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.RED_BOARD) {
//            trajB = trajB.lineTo(new Vector2d(startPose.getX(), -12))
//                    .lineTo(new Vector2d(40, -12));
//            if (propPos == TensorFlowDetection.PropPosition.LEFT) {
//                trajB = trajB.lineTo(new Vector2d(40, -28.25));
//            } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
//                trajB = trajB.lineTo(new Vector2d(40, -42.5));
//            } else {
//                trajB = trajB.lineTo(new Vector2d(40, -35));
//
//            }
//            trajB = trajB
//
//                    .addTemporalMarker(() -> {
//                        android.util.Log.i("PLACE PIXEL", "Placed pixel at Red Board");
//                    })
//                    .turnTo(0);
//
//        } else if (getRobot().getDrive().currentQuadrant() == TrajectoryDrive.Quadrant.BLUE_BOARD) {
//            trajB = trajB.lineTo(new Vector2d(startPose.getX(), 12))
//                    .lineTo(new Vector2d(40, 12));
//            if (propPos == TensorFlowDetection.PropPosition.LEFT) {
//                trajB = trajB.lineTo(new Vector2d(40, 28.25));
//            } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
//                trajB = trajB.lineTo(new Vector2d(40, 42.5));
//            } else {
//                trajB = trajB.lineTo(new Vector2d(40, 35));
//
//            }
//            trajB = trajB
//
//                    .turn(Math.toRadians(90 - startPose.getHeading()))
//                    //TODO:fix error with temporal marker because ryan is a dum dum
//                    .addTemporalMarker(() -> {
//                        android.util.Log.i("PLACE PIXEL", "Placed pixel at Blue Board");
//                    })
//                    .turnTo(0);
//
//
//        } else {
//            trajB = trajB.lineTo(new Vector2d(startPose.getX(), 12))
//                    .lineTo(new Vector2d(40, 12));
//            if (propPos == TensorFlowDetection.PropPosition.LEFT) {
//                trajB = trajB.lineTo(new Vector2d(40, 28.25));
//            } else if (propPos == TensorFlowDetection.PropPosition.RIGHT) {
//                trajB = trajB.lineTo(new Vector2d(40, 42.5));
//            } else {
//                trajB = trajB.lineTo(new Vector2d(40, 38.5));
//            }
//            trajB = trajB
//
//                    .addTemporalMarker(() -> {
//                        android.util.Log.i("PLACE PIXEL", "Placed pixel at Blue Board");
//                    })
//                    .turnTo(0);
//
//
//        }
//        if (getRobot().getClass() == PsiBot.class) {
//            trajB.turn(Math.toRadians(180));
//        }
//
//        TrajectorySequence traj = trajB.build();
//        Log.i("PROGRESS", "1");
//        GlobalOpMode.opMode.telemetry.log().add("1");
//        getRobot().getDrive().followTrajectorySequence(traj);
//        Log.i("PROGRESS", "2");
//        GlobalOpMode.opMode.telemetry.log().add("2");
//        while ((((PsiBot) getRobot()).armMotor.getCurrentPosition() >= -1000 || ((PsiBot) getRobot()).armMotor.getCurrentPosition() <= -1100))
//            if (getRobot().getClass() == PsiBot.class) {
//
//                ((PsiBot) getRobot()).armMotor.setTargetPosition(-1052);
//                ((PsiBot) getRobot()).armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ((PsiBot) getRobot()).armMotor.setPower(.1);
//                Log.i("PROGRESS", "4");
//
//            }
//        Log.i("PROGRESS", "3");

    }



}
