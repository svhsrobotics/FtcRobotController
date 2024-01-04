package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RoboticaBot extends Robot {

    protected RoboticaBot(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public TrajectoryDrive getDrive() {
        return null;
    }
}
