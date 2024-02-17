package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.RoboticaBot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;


@Autonomous
public class TestDriveForward extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        Robot robot = Robot.thisRobot(hardwareMap);



        android.util.Log.i("AUTO", "Waiting for start...");
        waitForStart();
        if (isStopRequested()) {
            android.util.Log.d("AUTO", "STOP requested after waitForStart(), returning early");
            return;
        }

        TrajectorySequenceBuilder trajB = robot.getDrive().trajectorySequenceBuilder(robot.getDrive().getPoseEstimate());
        trajB.forward(48);
        TrajectorySequence traj = trajB.build();
        robot.getDrive().followTrajectorySequence(traj);









    }


}

