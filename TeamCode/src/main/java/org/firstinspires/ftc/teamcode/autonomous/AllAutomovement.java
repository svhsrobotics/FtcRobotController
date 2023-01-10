package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Shared.PoleLocalize;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBotV2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.util.Logger;
//import org.firstinspires.ftc.teamcode.vision.TensorflowStandardSleeve;
import org.firstinspires.ftc.teamcode.util.PIController;
import org.firstinspires.ftc.teamcode.util.Timeout;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.pole.DoubleThresholdPipeline;

@Autonomous(name = "Autonomous", preselectTeleOp = "TeleOp")
public class AllAutomovement extends LinearOpMode {
    private final Logger logger = new Logger(telemetry, true);

    private PowerPlayBotV2 robot;
    private PoleLocalize drive;

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

    @SuppressWarnings("ConstantConditions")
    private void localizeOnPole(DoubleThresholdPipeline pipeline) {
        PIController poleLocalizationController = new PIController((0.06 / 8.0) / 2, 0);

        int settleCounter = 0;
        while (settleCounter < 3 && (opModeIsActive() || opModeInInit())) {
            double correctionPx = -pipeline.necessaryCorrection();
            logger.debug("px Correction: " + correctionPx);

            if (Math.abs(correctionPx) < 5) {
                // If we're really close to the target, increase the settle counter
                settleCounter++;
            } else {
                // If we're not close, reset the settle counter
                settleCounter = 0;
            }

            double correctionPower = poleLocalizationController.update(correctionPx);
            logger.debug("Power Correction: " + correctionPower);

            robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(-correctionPower);
            robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(-correctionPower);
            robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(correctionPower);
            robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(correctionPower);
        }

        // Stop the wheels when we're done (just in case)
        for (Drive drive : robot.drives.values()) {
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.setPower(0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PowerPlayBotV2(hardwareMap, logger);
        robot.initHardware();
        //drive = new PoleLocalize(robot, this, initializePoleDetection());

        DoubleThresholdPipeline pipeline = initializePoleDetection();

        while (opModeInInit()) {
            logger.debug("Calling localize on pole...");
            localizeOnPole(pipeline);
            sleep(5000);
        }
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

