package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.TestRobot;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.vision.TfodSleeve;

@Autonomous(name = "Autonomous")
public class Redwithredcorner extends LinearOpMode {
    private final Logger logger = new Logger(telemetry, true);

    @Override
    public void runOpMode() throws InterruptedException {
        TestRobot robot = new TestRobot(hardwareMap, logger);
        robot.initHardware();
//140 cm of working space
        Drive2 drive = new Drive2(robot, this);

        waitForStart();

        //state the case when testing

        TfodSleeve test = TfodSleeve.ONE;
        logger.info("sleeveresults " + test);
        switch(test) {
            case ONE:
                drive.navigationMonitorTicks(100, -100, 0, 10);
                drive.navigationMonitorTicks(100, 0, 145, 10);
                break;
            case TWO:
                drive.navigationMonitorTicks(100, 0, 145, 10);
                break;
            case THREE:
                drive.navigationMonitorTicks(100, 100, 0, 10);
                drive.navigationMonitorTicks(100, 0, 145, 10);
                break;
        }
    }
}
