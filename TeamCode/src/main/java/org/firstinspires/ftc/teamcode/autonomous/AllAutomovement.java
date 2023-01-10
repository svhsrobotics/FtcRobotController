package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Shared.Drive;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBotV2;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.Logger;
//import org.firstinspires.ftc.teamcode.vision.TensorflowStandardSleeve;
import org.firstinspires.ftc.teamcode.util.Timeout;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.TensorflowStandardSleeve;
import org.firstinspires.ftc.teamcode.vision.TfodSleeve;
import org.firstinspires.ftc.teamcode.vision.pole.DoubleThresholdPipeline;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Autonomous", preselectTeleOp = "TeleOp")
public class AllAutomovement extends LinearOpMode {
    private final Logger logger = new Logger(telemetry, true);

    private PowerPlayBotV2 robot;
    private Drive drive;

    private AprilTagPipeline initializeAprilTag() {
        AprilTagPipeline pipeline = new AprilTagPipeline();
        robot.camera.setPipeline(pipeline);
        if (!robot.camera.opened) {
            robot.camera.open(new Timeout(10));
        } else {
            robot.camera.resume();
        }
        return pipeline;
    }

    private DoubleThresholdPipeline initializePoleDetection() {
        DoubleThresholdPipeline pipeline = new DoubleThresholdPipeline(telemetry);

        // Tune the pipeline for yellow poles
        pipeline.minHue = 16.4;
        pipeline.maxHue = 36.6;
        pipeline.minBlue = 167.4;
        pipeline.maxBlue = 208.4;
        // Offset for center of robot
        pipeline.targetX = 120;

        robot.camera.setPipeline(pipeline);
        if (!robot.camera.opened) {
            robot.camera.open(new Timeout(10));
        } else {
            robot.camera.resume();
        }
        return pipeline;
    }

    private int detectTag(AprilTagPipeline pipeline) {
        logger.debug("Detecting tag...");
        // Wait for the camera to see a tag
        Timeout detectionTimeout = new Timeout(5);
        while (pipeline.getIds() == null && opModeIsActive() && !detectionTimeout.expired()) { Thread.yield(); }
        // null check to prevent crash
        int[] ids = pipeline.getIds();
        if (ids == null) {
            logger.warning("Could not find an AprilTag, falling back!");
            return 0;
        }
        return ids[0];
    }

    private void localizeOnPole() {
        //DoubleThresholdPipeline pipeline = initializePoleDetection();

        //while (opModeIsActive()) {
        while (opModeInInit()) {
            // Turn until centered
            //logger.debug("Correction: " + pipeline.necessaryCorrection());
            //logger.debug("Current Angle: " + drive.getAdjustedAngle());
            logger.debug("Target Angle: " + drive.mTargetAngle);
            double angle = drive.getAdjustedAngle();  //IMU angle converted to Euler angle (IMU may already deliver Euler angles)
            double angleError = drive.getEulerAngleDegrees(drive.mTargetAngle - angle);
            logger.debug("Angle Error: " + angleError);
            //drive.navigationMonitorTurn(drive.mTargetAngle - (0.1 * pipeline.necessaryCorrection()), false);
            drive.navigationMonitorTurn(drive.mTargetAngle);
            drive.ceaseMotion();

            sleep(4000);

            //drive.navigationMonitorTurn(drive.mTargetAngle, false);
            //drive.getAdjustedAngle();
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PowerPlayBotV2(hardwareMap, logger);
        robot.initHardware();
        drive = new Drive(robot, this, initializePoleDetection());

        localizeOnPole();
        // Start the AprilTagPipeline
        //AprilTagPipeline aprilTagPipeline = initializeAprilTag();
        /*
        // Wait for the start button to be pressed
        waitForStart();

        // Detect the tag
        int tagId = detectTag(aprilTagPipeline);
        logger.info("AprilTag Detected: " + tagId);

        // Pause the pipeline to save resources
        robot.camera.pause();

        // Get the cone in the pincher and get ready to drive
        robot.arm.pincher.expand();

        // Move forward so we don't scrape against the wall
        drive.navigationMonitorTicksAndCeaseMotion(3, 0, 2, 10);

        // Lift the preloaded cone up to drive height
        robot.arm.lift.setPreset(Arm.Lift.Preset.DRIVING);

        // Wait for it to arrive at drive height
        while (robot.arm.lift.isBusy() && !isStopRequested()) {
            Thread.yield();
        }

        // Move right past the ground station
        drive.navigationMonitorTicksAndCeaseMotion(3, 15, 0, 10);

        // Move forward
        drive.navigationMonitorTicksAndCeaseMotion(3, 0, 17, 10);

        // Turn to face our target direction
        drive.navigationMonitorTurn(60);

        // Move up to our special spot
        drive.navigationMonitorTicksPhi(3, 0, 6, 60, 10);
        drive.ceaseMotion();

        // Now it's time to drop the preloaded cone!
        // Raise up to med height so that our camera is not obstructed
        robot.arm.lift.setPreset(Arm.Lift.Preset.MEDIUM_POLE);

        // Wait for it to arrive at the height
        while (robot.arm.lift.isBusy() && !isStopRequested()) {
            Thread.yield();
        }

        // Center on the pole
        localizeOnPole();

        // Raise it up to high height
        robot.arm.lift.setPreset(Arm.Lift.Preset.HIGH_POLE);

        // Wait for it to arrive at the height
        while (robot.arm.lift.isBusy() && !isStopRequested()) {
            Thread.yield();
        }

        // Extend the reacher over the pole
        // TODO
        // Drop the cone
        robot.arm.pincher.contract();

         */
    }
}

