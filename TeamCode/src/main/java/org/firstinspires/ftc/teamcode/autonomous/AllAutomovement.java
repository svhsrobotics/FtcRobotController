package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Shared.Navigator;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBotV2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.util.Logger;
//import org.firstinspires.ftc.teamcode.vision.TensorflowStandardSleeve;
import org.firstinspires.ftc.teamcode.util.PIController;
import org.firstinspires.ftc.teamcode.util.Timeout;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.pole.DoubleThresholdPipeline;

@Config
@Autonomous(name = "Autonomous", preselectTeleOp = "TeleOp")
public class AllAutomovement extends LinearOpMode {
    private final Logger logger = new Logger(telemetry, true);

    private PowerPlayBotV2 robot;
    private Navigator navigator;

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
        pipeline.targetX = 160;

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

    @Config
    public static class RotationPID {
        public static double PGAIN = (0.06 / 8.0) / 2;
        public static double IGAIN = 0.0;
        public static double IWINDUPLIMIT = 3.0;
        public static int SETTLE_PX = 5;
        public static int TARGET = 140;
        public static int FRAME_SLEEP = 30;
    }

    @SuppressWarnings("ConstantConditions")
    private void localizeOnPole(DoubleThresholdPipeline pipeline) {
        PIController poleLocalizationController = new PIController(RotationPID.PGAIN, RotationPID.IGAIN, RotationPID.IWINDUPLIMIT);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        int settleCounter = 0;
        while (settleCounter < 3 && (opModeIsActive() || opModeInInit())) {
            TelemetryPacket packet = new TelemetryPacket();
            double x = pipeline.getPoleX();
            packet.put("x", x);
            if (x == -1.0) {
                // NO POLES: STOP RIGHT NOW
                for (Drive drive : robot.drives.values()) {
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    drive.setPower(0);
                }
                dashboard.sendTelemetryPacket(packet);
                sleep(RotationPID.FRAME_SLEEP);
                continue;
            }
            double correction = RotationPID.TARGET - x;
            packet.put("correction", correction);
            //logger.debug("px Correction: " + correctionPx);

            if (Math.abs(correction) < RotationPID.SETTLE_PX) {
                // If we're really close to the target, increase the settle counter
                settleCounter++;
                packet.put("settle", settleCounter);
            } else {
                // If we're not close, reset the settle counter
                settleCounter = 0;
            }

            double correctionPower = poleLocalizationController.update(correction);
            packet.put("power", correctionPower);

            dashboard.sendTelemetryPacket(packet);

            //logger.debug("Power Correction: " + correctionPower);

            robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(-correctionPower);
            robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(-correctionPower);
            robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(correctionPower);
            robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(correctionPower);

            sleep(RotationPID.FRAME_SLEEP);
        }

        // Stop the wheels when we're done (just in case)
        for (Drive drive : robot.drives.values()) {
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.setPower(0);
        }
    }

    @Config
    public static class DistancePID {
        public static double PGAIN = 0.06;
        public static double IGAIN = 0;
        public static double IWINDUP_LIMIT = 100.0;
        public static double SETTLE_LIMIT = 0.1;
        public static int TARGET = 26;
        public static int FRAME_SLEEP = 30;
        public static double SAFETY_MAX = 1.0;
        public static double MIN_WIDTH = 15.0;
        public static double CUTOFF = 30;
    }

    @SuppressWarnings("ConstantConditions")
    private void localizeDistance(DoubleThresholdPipeline pipeline) {
        PIController distanceController = new PIController(DistancePID.PGAIN, DistancePID.IGAIN, DistancePID.IWINDUP_LIMIT);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        int settleCounter = 0;
        while (settleCounter < 3 && (opModeIsActive() || opModeInInit())) {
            TelemetryPacket packet = new TelemetryPacket();
            double width = pipeline.getPoleWidth();
            packet.put("width", width);
            if (width == -1.0 || width < DistancePID.MIN_WIDTH) {
                // NO POLES: STOP RIGHT NOW
                for (Drive drive : robot.drives.values()) {
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    drive.setPower(0);
                }
                dashboard.sendTelemetryPacket(packet);
                sleep(DistancePID.FRAME_SLEEP);
                continue;
            }
            if (width > DistancePID.CUTOFF) {
                // This isn't realistic, we're getting a bad reading
                width = DistancePID.CUTOFF;
            }
            double correction = DistancePID.TARGET - width;
            packet.put("correction", correction);
            double widthPowerCorrection = distanceController.update(correction);
            packet.put("power", widthPowerCorrection);

            dashboard.sendTelemetryPacket(packet);


            if (Math.abs(widthPowerCorrection) < DistancePID.SETTLE_LIMIT) {
                // If we're really close to the target, increase the settle counter
                settleCounter++;
                //logger.debug("Settle Counter: " + settleCounter);
            } else {
                // If we're not close, reset the settle counter
                settleCounter = 0;
            }
            //logger.debug("Width Power Correction: " + widthPowerCorrection);

            widthPowerCorrection = Range.clip(widthPowerCorrection, -DistancePID.SAFETY_MAX, DistancePID.SAFETY_MAX);
            robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(widthPowerCorrection);
            robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(widthPowerCorrection);
            robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(widthPowerCorrection);
            robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(widthPowerCorrection);

            sleep(DistancePID.FRAME_SLEEP);
        }

        // Stop the wheels when we're done (just in case)
        for (Drive drive : robot.drives.values()) {
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.setPower(0);
        }
    }

    public static double AUTO_SPEED = 20.0;
    public static int REACH_LENGTH = 1800;
    public static double NEXT_DEG = 90.0;
    public static double TURN_DEG = 57.0;

    public static boolean DO_AUTO_PATH = true;
    public static boolean DO_DISTANCE = true;
    public static boolean DO_ROTATION = true;
    public static boolean DO_PARKING = true;
    public static boolean LOOP_ROTATION = false;
    public static boolean LOOP_DISTANCE = false;

   // public static double X_MODIFIER = 0.0;
    public static double Y_PARK = 0.0;

    public static int LOOP_SLEEP = 5000;

    public static double MAGIC_CONVERSION = .1;

    public static double FORWARD = 17;

    public static boolean LEFT_SIDE_MIRROR = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PowerPlayBotV2(hardwareMap, logger);
        robot.initHardware();
        navigator = new Navigator(robot, this);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        AprilTagPipeline aprilTagPipeline = new AprilTagPipeline();
        //SwitchablePipeline pipeline = new SwitchablePipeline(aprilTagPipeline); //TODO: Remove no longer necessary, use setPipeline
        robot.camera.setPipeline(aprilTagPipeline);
        if (!robot.camera.opened) {
            robot.camera.open(new Timeout(10));
        } else {
            robot.camera.resume();
        }

        // Wait for the start button to be pressed
        waitForStart();

        int tagId = 0;
        if (DO_AUTO_PATH) {
            // Detect the tag
            tagId = detectTag(aprilTagPipeline);
            logger.info("AprilTag Detected: " + tagId);
        }

        DoubleThresholdPipeline poleDetectionPipeline = null;
        if (DO_DISTANCE || DO_ROTATION || LOOP_DISTANCE || LOOP_ROTATION) {
            poleDetectionPipeline = initializePoleDetection();
            robot.camera.setPipeline(poleDetectionPipeline);
        }

        if (isStopRequested()) return;

        if (DO_AUTO_PATH) {

            // Pause the pipeline to save resources
            // robot.camera.pause();

            // Get the cone in the pincher and get ready to drive
            robot.arm.pincher.expand();

            // Move forward so we don't scrape against the wall
            navigator.navigationMonitorTicksAndCeaseMotion(AUTO_SPEED, 0, 1, 10);

            // Lift the preloaded cone up to drive height
            robot.arm.lift.setPreset(Arm.Lift.Preset.DRIVING);

            // Wait for it to arrive at drive height
            while (robot.arm.lift.isBusy() && !isStopRequested()) {
                Thread.yield();
            }

            if (LEFT_SIDE_MIRROR) {
                // Move left past the ground station
                navigator.navigationMonitorTicksAndCeaseMotion(AUTO_SPEED, -15, 0, 10);
            } else {
                // Move right past the ground station
                navigator.navigationMonitorTicksAndCeaseMotion(AUTO_SPEED, 15, 0, 10);
            }

            // Move forward
            navigator.navigationMonitorTicksAndCeaseMotion(AUTO_SPEED, 0, FORWARD, 10);

            if (LEFT_SIDE_MIRROR) {
                // Turn to face our target direction
                navigator.navigationMonitorTurn(-TURN_DEG);
                // Move up to our special spot
                navigator.navigationMonitorTicksPhi(AUTO_SPEED, 0, 5.5, -TURN_DEG, 10);
                navigator.ceaseMotion();
            } else {
                // Turn to face our target direction
                navigator.navigationMonitorTurn(TURN_DEG);
                // Move up to our special spot
                navigator.navigationMonitorTicksPhi(AUTO_SPEED, 0, 5.5, TURN_DEG, 10);
                navigator.ceaseMotion();
            }

            // Now it's time to drop the preloaded cone!
            // Raise up to med height so that our camera is not obstructed
            robot.arm.lift.setPreset(Arm.Lift.Preset.MEDIUM_POLE);

            // Wait for it to arrive at the height
            while (robot.arm.lift.isBusy() && !isStopRequested()) {
                Thread.yield();
            }
        }


        if (DO_ROTATION) {
            localizeOnPole(poleDetectionPipeline);
        }
        if (DO_DISTANCE) {
            localizeDistance(poleDetectionPipeline);
        }
        if (DO_ROTATION) {
            localizeOnPole(poleDetectionPipeline);
        }

        while (LOOP_DISTANCE && opModeIsActive()) {
            localizeDistance(poleDetectionPipeline);
            sleep(LOOP_SLEEP);
        }
        while (LOOP_ROTATION && opModeIsActive()) {
            localizeOnPole(poleDetectionPipeline);
            sleep(LOOP_SLEEP);
        }

        if (DO_AUTO_PATH && !isStopRequested()) {

            // Raise it up to high height
            robot.arm.lift.setPreset(Arm.Lift.Preset.HIGH_POLE);

            // Wait for it to arrive at the height
            while (robot.arm.lift.isBusy() && !isStopRequested()) {
                Thread.yield();
            }

            robot.arm.reacher.setTargetPosition(REACH_LENGTH);

            while (robot.arm.reacher.isBusy() && !isStopRequested()) {}

            if (isStopRequested()) {
                robot.arm.reacher.setTargetPosition(0);
                return;
            }

            robot.arm.pincher.contract();

            sleep(500);

            //sleep(5000);

            robot.arm.reacher.setTargetPosition(0);

            //navigator.navigationMonitorTurn(NEXT_DEG);
           // navigator.ceaseMotion();

            while (robot.arm.reacher.isBusy() && !isStopRequested()) {}

            //robot.arm.lift.setPreset(Arm.Lift.Preset.MEDIUM_POLE);

        }

        if (DO_PARKING && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            double angle = navigator.getImuAngle();
            packet.put("IMU Angle", angle);

            // Invert constants for left side
            double next_deg = NEXT_DEG;
            double magic_conversion = MAGIC_CONVERSION;
            if (LEFT_SIDE_MIRROR) next_deg = -NEXT_DEG;
            if (LEFT_SIDE_MIRROR) magic_conversion = -MAGIC_CONVERSION;

            navigator.navigationMonitorTurn(next_deg);
            navigator.ceaseMotion();

            robot.arm.lift.setPower(0);

            double error = -(12 - (26 * Math.cos(angle)));
            packet.put("Error", error);
            packet.put("Converted", error * magic_conversion);
            dashboard.sendTelemetryPacket(packet);
            navigator.navigationMonitorTicksPhi(AUTO_SPEED, error * magic_conversion, Y_PARK, next_deg, 10);
            navigator.ceaseMotion();
        }
        while (!isStopRequested()) {}
    }
}

