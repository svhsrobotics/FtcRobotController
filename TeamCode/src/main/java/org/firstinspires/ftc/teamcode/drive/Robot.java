package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;

public abstract class Robot {
    public abstract TrajectoryDrive getDrive();
    protected Robot(HardwareMap hardwareMap) {
        // Any shared initialization goes here
    }

    public abstract AprilTagCamera[] getCameras();

    /**
     * Sets the purple pixel on the robot;
     * true: dropped;
     * false: closed
     */
    public abstract void dropPurplePixel(boolean state);
}
