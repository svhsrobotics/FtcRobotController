package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Navigator;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBotV2;
import org.firstinspires.ftc.teamcode.util.Logger;

@Autonomous
public class Rotation extends LinearOpMode {

    Logger logger = new Logger(telemetry);
    @Override
    public void runOpMode() throws InterruptedException {
        PowerPlayBotV2 robot = new PowerPlayBotV2(hardwareMap, logger);
        robot.initHardware();
        Navigator navigator = new Navigator(robot, this);

        waitForStart();

        //drive.navigationMonitorTicksPhi(0, 1, 0, 90, 10);
        navigator.navigationMonitorTurn(90);
        navigator.ceaseMotion();
    }
}
