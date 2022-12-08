package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBotV2;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.vision.TensorflowStandardSleeve;
import org.firstinspires.ftc.teamcode.vision.TfodSleeve;

@Autonomous(name = "Autonomous")
public class AllAutomovement extends LinearOpMode {
    private final Logger logger = new Logger(telemetry, true);

    @Override
    public void runOpMode() throws InterruptedException {
        PowerPlayBotV2 robot = new PowerPlayBotV2(hardwareMap, logger);
        robot.initHardware();
//140 cm of working space
        Drive2 drive = new Drive2(robot, this);

        waitForStart();
//        drive.navigationMonitorTicks(100.0, 5.0, 0.0, 10);//5
//        drive.navigationMonitorTicks(95.0, 5.0, 0.0, 10);//10
//        drive.navigationMonitorTicks(90.0, 5.0, 0.0, 10);//15
//        drive.navigationMonitorTicks(80.0, 5.0, 0.0, 10);//20
//        drive.navigationMonitorTicks(70.0, 5.0, 0.0, 10);//25
//        drive.navigationMonitorTicks(60.0, 5.0, 0.0, 10);//30
//        drive.navigationMonitorTicks(50.0, 5.0, 0.0, 10);//35
//        drive.navigationMonitorTicks(40.0, 5.0, 0.0, 10);//40
//        drive.navigationMonitorTicks(30.0, 5.0, 0.0, 10);//45
robot.arm.pincher.expand();
        //state the case when testing
        TensorflowStandardSleeve tensor = new TensorflowStandardSleeve(this);
        TfodSleeve detected = tensor.scanStandardSleeve();

        logger.info("Sleeve Detected: " + detected);
        switch(detected) {
            case ONE:
                drive.navigationMonitorTicks(12,0,3.5,10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(15,-47,0,10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(25, 0, 66.5, 10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(13, 0, 25.5, 10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(20, 30.5, 0, 10);
                robot.arm.pincher.contract();
                drive.ceaseMotion();
                drive.navigationMonitorTicks(20,-1,0,10);
                drive.navigationMonitorTicks(20, 65.5, 0, 10);
                drive.ceaseMotion();
                break;
            case TWO:
                drive.navigationMonitorTicks(12,0,3.5,10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(15,-47,0,10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(25, 0, 66.5, 10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(13, 0, 25.5, 10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(20, 30.5, 0, 10);
                robot.arm.pincher.contract();
                drive.ceaseMotion();
                drive.navigationMonitorTicks(20,-1,0,10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(20, 38.5, 0, 10);
                drive.ceaseMotion();
//                drive.navigationMonitorTicksPhi(1, 0, 34.5, -90, 10);
//                robot.arm.pincher.expand();
//                drive.ceaseMotion();
//                robot.arm.pincher.contract();
//                drive.ceaseMotion();
//                drive.navigationMonitorTicksPhi(1, 0, 31.5, 90, 10);
//                drive.ceaseMotion();


                //drive.navigationMonitorTicks(10, 25, 0, 10);
               // drive.navigationMonitorTicksPhi(10, 47, 0, -90, 10);
                break;
            case THREE:
                drive.navigationMonitorTicks(12,0,3.5,10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(15,-47,0,10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(25, 0, 66.5, 10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(13, 0, 25.5, 10);
                drive.ceaseMotion();
                drive.navigationMonitorTicks(20, 30.5, 0, 10);
                robot.arm.pincher.contract();
                drive.ceaseMotion();
                drive.navigationMonitorTicks(20, -30.5, 0, 10);
                drive.ceaseMotion();
                break;
        }
    }
}

