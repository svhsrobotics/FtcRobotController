package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TopLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevBlinkinLedDriver lights = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }
}
