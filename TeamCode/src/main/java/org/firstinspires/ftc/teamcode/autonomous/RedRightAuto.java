package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RedRightAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(16, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence autoDrive = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(16, -36))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(52, -36))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(autoDrive);
    }


}
