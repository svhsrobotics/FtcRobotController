package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBot;
import org.firstinspires.ftc.teamcode.robot.TestRobot;
import org.firstinspires.ftc.teamcode.robot.hardware.Grabber;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous
public class TopLeft extends LinearOpMode {
    private final Logger logger = new Logger(telemetry, true);

    @Override
    public void runOpMode() throws InterruptedException {
        PowerPlayBot robot = new PowerPlayBot(hardwareMap, logger);
        robot.initHardware();

        Drive2 drive = new Drive2(robot, this);

        waitForStart();
        robot.grabber.setGrabberPosition(Grabber.Positions.SETFOURCONE);
        while (!isStopRequested()) {

        }
        //drive.navigationMonitorTicks(10, 10, 10, 10);
    }
}
