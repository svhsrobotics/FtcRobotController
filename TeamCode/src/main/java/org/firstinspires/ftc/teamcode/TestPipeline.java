package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBotV2;
import org.firstinspires.ftc.teamcode.robot.Robot;
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

        waitForStart();

        while (opModeIsActive()) {
            // Tune for red cones
            pipeline.minHue = 174.0;
            pipeline.maxHue = 8.0;
            pipeline.minBlue = 147.3;
            pipeline.maxBlue = 184.2;

            // Turn until centered
            while (pipeline.necessaryCorrection() != 0) {
                //
            }
        }


        // Turn until centered


        /*while (opModeInInit()) {

            if (pipeline.necessaryCorrection() > 5) {
                // Right
                robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(.1);
                robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(-.1);
                robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(-.1);
                robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(.1);
                //drive.navigationMonitorTicksPhi(5, 1, 0, drive.getAdjustedAngle(), 1);
            } else if (pipeline.necessaryCorrection() < -5) {
                // Left
                robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(-.1);
                robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(.1);
                robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(.1);
                robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(-.1);
                //drive.navigationMonitorTicksPhi(5, -1, 0, drive.getAdjustedAngle(), 1);
            } else {
                robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(0);
                robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(0);
                robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(0);
                robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(0);
                //drive.navigationMonitorTicks(0, 9999, 99999, 1);
                //drive.ceaseMotion();
            }
        }*/
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
