package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Left")
public class Left extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AllAutomovement.LEFT_SIDE_MIRROR = true;
        AllAutomovement auto = new AllAutomovement(this);
        auto.runOpMode();
    }
}
