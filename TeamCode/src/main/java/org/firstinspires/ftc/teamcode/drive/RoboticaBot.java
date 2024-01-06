package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;

public class RoboticaBot extends Robot {

    protected RoboticaBot(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public AprilTagCamera[] getCameras() {
        return new AprilTagCamera[0];
    }

    @Override
    public void dropPurplePixel(boolean state) {
        // TODO
    }

    @Override
    public TrajectoryDrive getDrive() {
        return null;
    }
}
