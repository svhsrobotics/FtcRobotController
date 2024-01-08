package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class DriveShimShim {
    private final DriveShim driveShim;
    public TrajectorySequence currentTrajectorySequence;
    public DriveShimShim(DriveShim driveShim) {
        this.driveShim = driveShim;
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        currentTrajectorySequence = trajectorySequence;
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return driveShim.trajectorySequenceBuilder(startPose);
    }

    public Pose2d getPoseEstimate() {
        return driveShim.getPoseEstimate();
    }

    public void setPoseEstimate(Pose2d poseEstimate) {
        driveShim.setPoseEstimate(poseEstimate);
    }

    public static TrajectoryDrive.Quadrant whichQuadrant(Pose2d pose) {
        if (pose.getX() < 0) {
            if (pose.getY() < 0) {
                return TrajectoryDrive.Quadrant.RED_AUDIENCE;
            } else {
                return TrajectoryDrive.Quadrant.BLUE_AUDIENCE;
            }
        } else {
            if (pose.getY() < 0) {
                return TrajectoryDrive.Quadrant.RED_BOARD;
            } else {
                return TrajectoryDrive.Quadrant.BLUE_BOARD;
            }
        }
    }

    public TrajectoryDrive.Quadrant currentQuadrant() {
        return whichQuadrant(getPoseEstimate());
    }

}
