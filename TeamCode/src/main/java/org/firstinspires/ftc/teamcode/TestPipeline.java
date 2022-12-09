package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBotV2;
import org.firstinspires.ftc.teamcode.vision.pole.DoubleThresholdPipeline;

@TeleOp(name = "Test Pipeline")
public class TestPipeline extends LinearOpMode {

    @Override
    public void runOpMode() {
        PowerPlayBotV2 robot = new PowerPlayBotV2(hardwareMap);
        robot.initHardware();

        Drive2 drive = new Drive2(robot, this);

        DoubleThresholdPipeline pipeline = new DoubleThresholdPipeline(telemetry);

        robot.camera.setPipeline(pipeline);

        robot.camera.open();

        while (opModeInInit()) {

            if (pipeline.necessaryCorrection() > 5) {
                // Right
                drive.navigationMonitorTicksPhi(5, 1, 0, drive.getAdjustedAngle(), 1);
            } else if (pipeline.necessaryCorrection() < -5) {
                // Left
                drive.navigationMonitorTicksPhi(5, -1, 0, drive.getAdjustedAngle(), 1);
            } else {
                //drive.navigationMonitorTicks(0, 9999, 99999, 1);
                drive.ceaseMotion();
            }
        }
        /*waitForStart();


        while (opModeIsActive()) {
            if (pipeline.necessaryCorrection() > 1) {
                drive.navigationMonitorTicks(10, -1, 0,1);
            } else if (pipeline.necessaryCorrection() < 1) {
                drive.navigationMonitorTicks(10, 1, 0, 1);
            } else {
                drive.ceaseMotion();
            }
            telemetry.addData("Correction", pipeline.necessaryCorrection());
            telemetry.update();
            // Wait for stop to be pressed
        }*/

    }
}
