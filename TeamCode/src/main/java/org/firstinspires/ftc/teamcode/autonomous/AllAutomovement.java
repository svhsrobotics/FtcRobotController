package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Drive;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBotV2;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.Logger;
//import org.firstinspires.ftc.teamcode.vision.TensorflowStandardSleeve;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.TensorflowStandardSleeve;
import org.firstinspires.ftc.teamcode.vision.TfodSleeve;

@Autonomous(name = "Autonomous", preselectTeleOp = "TeleOp")
public class AllAutomovement extends LinearOpMode {
    private final Logger logger = new Logger(telemetry, true);

    @Override
    public void runOpMode() throws InterruptedException {
        PowerPlayBotV2 robot = new PowerPlayBotV2(hardwareMap, logger);
        robot.initHardware();
        Drive drive = new Drive(robot, this);

        AprilTagPipeline pipeline = new AprilTagPipeline();
        robot.camera.setPipeline(pipeline);
        robot.camera.open();

        waitForStart();

        while (pipeline.getIds() == null) {};

        int detected = pipeline.getIds()[0];
        robot.camera.close();

        logger.info("Sleeve Detected: " + detected);
        // Get the cone in the pincher and get ready to drive
        robot.arm.pincher.expand();
        // Move up to the pole
        drive.navigationMonitorTicks(12,0,3.5,10);
        robot.arm.lift.setPreset(Arm.Lift.Preset.DRIVING);
        drive.ceaseMotion();
        drive.navigationMonitorTicks(15,32 + 30,0,10);
        drive.ceaseMotion();
        drive.navigationMonitorTicks(25, 0, 66.5, 10);
        drive.ceaseMotion();
        drive.navigationMonitorTicks(13, 0, 18, 10);
        drive.ceaseMotion();
        drive.navigationMonitorTicks(20, -25, 0, 10);
        drive.ceaseMotion();
        //drive.na
        drive.navigationMonitorTicksPhi(0, 10, 0 ,70,1);

        robot.arm.lift.setPreset(Arm.Lift.Preset.HIGH_POLE);
        while (robot.arm.lift.isBusy() && !isStopRequested()) {}
        robot.arm.reacher.setTargetPosition(2000);
        while (robot.arm.reacher.isBusy() && !isStopRequested()) {}
        robot.arm.pincher.contract();
        robot.arm.reacher.setTargetPosition(0);
        while (robot.arm.reacher.isBusy() && !isStopRequested()) {}
        robot.arm.lift.setPreset(Arm.Lift.Preset.DRIVING);
        while (robot.arm.lift.isBusy() && !isStopRequested()) {}
        // This is where the pole would be dropped


        switch(detected) {
            case 16: // This is the rightmost slot
                drive.navigationMonitorTicks(20, 70.5, 0, 10);
                drive.ceaseMotion();
                break;
            case 15: // This is the center slot
                drive.navigationMonitorTicks(20, 20, 0, 10);
                drive.ceaseMotion();
                break;
            case 14: // This is the leftmost slot
                drive.navigationMonitorTicks(20, -32, 0, 10);
                drive.ceaseMotion();
                break;
        }
    }
}

