package com.example.meepmeeptesting;

import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class Robot {
    private final DriveShimShim driveShim;
    public DriveShimShim getDrive() {
        return driveShim;
    }
    protected Robot(DriveShim driveShim) {
        // Any shared initialization goes here
        this.driveShim = new DriveShimShim(driveShim);
    }

//    public AprilTagCamera[] getCameras() {
//        return null;
//    }

    /**
     * Sets the purple pixel on the robot;
     * true: dropped;
     * false: closed
     */
    public void dropPurplePixel(boolean state) {
        // Do nothing
    }

    public TrajectorySequence getCurrentTrajectorySequence() {
        return driveShim.currentTrajectorySequence;
    }
}
