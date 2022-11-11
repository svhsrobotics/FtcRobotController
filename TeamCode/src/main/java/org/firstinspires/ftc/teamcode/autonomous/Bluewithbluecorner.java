package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.TestRobot;
import org.firstinspires.ftc.teamcode.util.Logger;

@Disabled
@Autonomous
public class Bluewithbluecorner extends LinearOpMode {
    private final Logger logger = new Logger(telemetry, true);

    @Override
    public void runOpMode() throws InterruptedException {
        TestRobot robot = new TestRobot(hardwareMap, logger);
        robot.initHardware();
//117 cm of working space or 46 inches
        Drive2 drive = new Drive2(robot, this);

        waitForStart();

        drive.navigationMonitorTicks(100, -100, 0, 10);
        drive.navigationMonitorTicks(100, 0, 145, 10);

        // drive.navigationMonitorTicks(100, 57, 0, 10);
//        drive.ceaseMotion();
//        drive.navigationMonitorTicks(100, 0, 105, 10);
//        drive.ceaseMotion();
//        sleep(2000);
//        drive.ceaseMotion();
//        drive.navigationMonitorTicks(100, 59, 0, 10);
//        drive.ceaseMotion();
//        drive.navigationMonitorTicks(100, 0, 105, 10);
//        drive.ceaseMotion();
//        drive.navigationMonitorTicksPhi(0, 1000, 1000, -90, 10);
//        drive.ceaseMotion();

        logger.info("a");
        String test = "none";
        switch(test) {
            case "one":
                drive.navigationMonitorTicks(100, -100, 0, 10);
                drive.navigationMonitorTicks(100, 0, 145, 10);
                break;
            case "two":
                drive.navigationMonitorTicks(100, 0, 145, 10);
                break;
            case "three":
                drive.navigationMonitorTicks(100, 100, 0, 10);
                drive.navigationMonitorTicks(100, 0, 145, 10);
                break;
        }
    }
}
