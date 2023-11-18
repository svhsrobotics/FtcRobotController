package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class AprilTagCamera {
    public WebcamName webcamName;
    private final double offset;
    private final double angle;

    public AprilTagCamera(WebcamName webcamName, double offset, double angle) {
        this.webcamName = webcamName;
        this.offset = offset;
        this.angle = angle;
    }

    public Pose2d translatePose(Pose2d inputPose) {
        double offsetX = offset * Math.cos(inputPose.getHeading());
        double offsetY = offset * Math.sin(inputPose.getHeading());

        android.util.Log.i("AprilTag Test", "Camera offset: " + offsetX + ", " + offsetY);

        double robotX = inputPose.getX() - offsetX;
        double robotY = inputPose.getY() - offsetY;
        double robotHeading = inputPose.getHeading() - angle;

        return new Pose2d(robotX, robotY, robotHeading);
    }
}
