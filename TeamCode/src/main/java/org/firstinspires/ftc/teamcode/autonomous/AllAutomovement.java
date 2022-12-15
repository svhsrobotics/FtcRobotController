package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBotV2;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.Logger;
//import org.firstinspires.ftc.teamcode.vision.TensorflowStandardSleeve;
import org.firstinspires.ftc.teamcode.util.Timeout;
import org.firstinspires.ftc.teamcode.vision.TensorflowGearPrRobotCOMPETITION;
import org.firstinspires.ftc.teamcode.vision.TfodSleeve;

@Autonomous(name = "Autonomous")
public class AllAutomovement extends LinearOpMode {
    private final Logger logger = new Logger(telemetry, true);

    @Override
    public void runOpMode() throws InterruptedException {
        PowerPlayBotV2 robot = new PowerPlayBotV2(hardwareMap, logger);
        robot.initHardware();
        Drive2 drive = new Drive2(robot, this);

//        drive.navigationMonitorTicks(100.0, 5.0, 0.0, 10);//5
//        drive.navigationMonitorTicks(95.0, 5.0, 0.0, 10);//10
//        drive.navigationMonitorTicks(90.0, 5.0, 0.0, 10);//15
//        drive.navigationMonitorTicks(80.0, 5.0, 0.0, 10);//20
//        drive.navigationMonitorTicks(70.0, 5.0, 0.0, 10);//25
//        drive.navigationMonitorTicks(60.0, 5.0, 0.0, 10);//30
//        drive.navigationMonitorTicks(50.0, 5.0, 0.0, 10);//35
//        drive.navigationMonitorTicks(40.0, 5.0, 0.0, 10);//40
//        drive.navigationMonitorTicks(30.0, 5.0, 0.0, 10);//45
        //state the case when testing
        TensorflowGearPrRobotCOMPETITION tensor = new TensorflowGearPrRobotCOMPETITION(this);
        TfodSleeve detected = tensor.scanStandardSleeve();

        waitForStart();
        robot.arm.pincher.expand();
        robot.arm.lift.setPreset(Arm.Lift.Preset.DRIVING);


        logger.info("Sleeve Detected: " + detected);

        // Move up to the pole
        drive.navigationMonitorTicks(12,0,3.5,10);
        drive.ceaseMotion();
        drive.navigationMonitorTicks(15,-32 - 10,0,10);
        drive.ceaseMotion();
        drive.navigationMonitorTicks(25, 0, 66.5, 10);
        drive.ceaseMotion();
        drive.navigationMonitorTicks(13, 0, 25.5, 10);
        drive.ceaseMotion();
        drive.navigationMonitorTicks(20, 32, 0, 10);
        drive.ceaseMotion();

        // This is where the pole would be dropped


        switch(detected) {
            case THREE: // This is the rightmost slot
                drive.navigationMonitorTicks(20, 70.5, 0, 10);
                drive.ceaseMotion();
                break;
            case TWO: // This is the center slot
                drive.navigationMonitorTicks(20, 20, 0, 10);
                drive.ceaseMotion();
                break;
            case ONE: // This is the leftmost slot
                drive.navigationMonitorTicks(20, -32, 0, 10);
                drive.ceaseMotion();
                break;
        }
    }
}

