package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RedLeftAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-34, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence autoDrive = drive.trajectorySequenceBuilder(startPose)

                .forward(24)
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .waitSeconds(3)
                .turn(Math.toRadians(-180))
                .forward(80)
                        .build();




           /*
                .lineTo(new Vector2d(-34, -36))
                .strafeTo(new Vector2d(48, -36))
                .build();
           */
           /*     .splineTo(new Vector2d(-34, -30), Math.toRadians(90))
                .splineTo(new Vector2d(34, -30), Math.toRadians(-90))
                .splineTo(new Vector2d(56, -56),Math.toRadians(0) )
                .build();
           */

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(autoDrive);
    }
}




