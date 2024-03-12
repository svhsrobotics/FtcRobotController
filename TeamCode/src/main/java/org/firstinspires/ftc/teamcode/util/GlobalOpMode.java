package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;

public class GlobalOpMode {
    public static LinearOpMode opMode = null;
    public static Pose2d lastPose = null;
    public static Robot robot = null;
}
