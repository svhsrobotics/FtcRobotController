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

@Autonomous(name = "Autonomous", preselectTeleOp = "TeleOp")
public class AllAutomovement extends LinearOpMode {
    private final Logger logger = new Logger(telemetry, true);

    private final PowerPlayBotV2 robot = new PowerPlayBotV2(hardwareMap, logger);
    private final Drive drive = new Drive(robot, this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHardware();

        // Open the camera and start the pipeline
        AprilTagPipeline pipeline = new AprilTagPipeline();
        robot.camera.setPipeline(pipeline);
        robot.camera.open(new Timeout(10));
        // If it timed out, then it will have returned early
        if (robot.camera.opened) {
            logger.debug("Camera opened!");
        } else {
            logger.warning("Could not open camera! (See logcat for error code)");
        }


        // Wait for the start button to be pressed
        waitForStart();

        logger.debug("Waiting to see an AprilTag...");
        // Wait for the camera to see a tag
        Timeout detectionTimeout = new Timeout(5);
        while (pipeline.getIds() == null && opModeIsActive() && !detectionTimeout.expired()) { Thread.yield(); }
        // null check to prevent crash
        int[] ids = pipeline.getIds();
        if (ids == null) {
            logger.warning("Could not find an AprilTag, falling back to 0!");
            ids = new int[]{0};
        }
        // Get the tag ID
        int detected = ids[0];
        // Pause the pipeline to save resources
        robot.camera.pause();

        logger.info("AprilTag Detected: " + detected);

        // Get the cone in the pincher and get ready to drive
        robot.arm.pincher.expand();
        // Move forward so we don't scrape against the wall
        drive.navigationMonitorTicksAndCeaseMotion(3, 0, 2, 10);
        // Lift the preloaded cone up to drive height
        robot.arm.lift.setPreset(Arm.Lift.Preset.DRIVING);
        // Wait for it to arrive at drive height
        while (robot.arm.lift.isBusy() && !isStopRequested()) { Thread.yield(); }
        // Move right past the ground station
        drive.navigationMonitorTicksAndCeaseMotion(3,15,0,10);
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
        while (robot.arm.lift.isBusy() && !isStopRequested()) { Thread.yield(); }
        // Center on the pole

        // TODO
        // Raise it up to high height
        robot.arm.lift.setPreset(Arm.Lift.Preset.HIGH_POLE);
        // Wait for it to arrive at the height
        while (robot.arm.lift.isBusy() && !isStopRequested()) { Thread.yield(); }
        // Extend the reacher over the pole
        // TODO
        // Drop the cone
        robot.arm.pincher.contract();

        //drive.navigationMonitorTicksAndCeaseMotion(12,0,3.5,10);
        //robot.arm.lift.setPreset(Arm.Lift.Preset.DRIVING);
        //drive.navigationMonitorTicksAndCeaseMotion(15,32,0,10);
        /*drive.navigationMonitorTicksAndCeaseMotion(25, 0, 66.5, 10);
        drive.navigationMonitorTicksAndCeaseMotion(13, 0, 18, 10);
        drive.navigationMonitorTicksAndCeaseMotion(20, -25, 0, 10);
        drive.navigationMonitorTurn(70);*/
        // Arm should move up and outwards to the first pole and drop a cone then hopefully not hit anything on its way down
        //robot.arm.lift.setPreset(Arm.Lift.Preset.HIGH_POLE);
        //while (robot.arm.lift.isBusy() && !isStopRequested()) {}
        //while (opModeIsActive()) {}
//        robot.arm.reacher.setTargetPosition(2000);
//        while (robot.arm.reacher.isBusy() && !isStopRequested()) {}
//        robot.arm.pincher.contract();
//        robot.arm.reacher.setTargetPosition(0);
//        while (robot.arm.reacher.isBusy() && !isStopRequested()) {}
//        robot.arm.lift.setPreset(Arm.Lift.Preset.DRIVING);
//        while (robot.arm.lift.isBusy() && !isStopRequested()) {}
        // This is where the pole would be dropped
//        Now the car should turn and get a cone from the cone pile and then come back
//        drive.navigationMonitorTurn(0);
//        drive.navigationMonitorTurn(-90);
//        robot.arm.pincher.contract();
//        robot.arm.lift.setTargetPosition(600);
//        while (robot.arm.lift.isBusy() && !isStopRequested()) {}
//        robot.arm.reacher.setTargetPosition(500);
//        while (robot.arm.reacher.isBusy() && !isStopRequested()) {}
//        robot.arm.lift.setTargetPosition(500);
//        while (robot.arm.lift.isBusy() && !isStopRequested()) {}
//        robot.arm.pincher.expand();
//        robot.arm.lift.setTargetPosition(600);
//        while (robot.arm.lift.isBusy() && !isStopRequested()) {}
//        robot.arm.reacher.setTargetPosition(0);
//        while (robot.arm.reacher.isBusy() && !isStopRequested()) {}



//        switch(detected) {
//            case 16: // This is the rightmost slot
//                drive.navigationMonitorTicks(20, 70.5, 0, 10);
//                drive.ceaseMotion();
//                break;
//            case 15: // This is the center slot
//                drive.navigationMonitorTicks(20, 20, 0, 10);
//                drive.ceaseMotion();
//                break;
//            case 14: // This is the leftmost slot
//                drive.navigationMonitorTicks(20, -32, 0, 10);
//                drive.ceaseMotion();
//                break;
//        }
    }

    private void localizeOnPole() {

    }
}

