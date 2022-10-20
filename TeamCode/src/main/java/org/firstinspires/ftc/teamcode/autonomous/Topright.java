package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Shared.Drive2;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Arm;
import org.firstinspires.ftc.teamcode.robot.hardware.Webcam;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.Configurator;
import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;

public class Topright extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RevBlinkinLedDriver lights = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        // Get the webcam from the hardware map
        Webcam webcam = new Webcam("Webcam 1", hardwareMap);
        Servo cap;
        CRServo rightCarousel;
        CRServo leftCarousel;
        cap = hardwareMap.get(Servo.class, "cap");
        // Load the configuration
        Configuration config = Configurator.load();
        if (config.target == null) {
            // This is an extra warning to make debugging errors a little easier
            telemetry.log().add("WARNING: WAS NOT CALIBRATED");
            // So we don't get a NullPointerException later on...
            config.target = new HSVColor(0.0, 0.0, 0.0);
        }

        // Create a detector pipeline from the config
        TeamElementDetector detector = new TeamElementDetector(config);
        // Tell the camera to start using the pipeline
        webcam.setPipeline(detector);
        // Create the robot from the hardware map
        Robot robot = new Robot(hardwareMap);
        rightCarousel = hardwareMap.get(CRServo.class, "rightCarousel");
        leftCarousel = hardwareMap.get(CRServo.class, "leftCarousel");
        // Initialize the hardware
        robot.initHardware();
        // Reset the arm encoder to 0 to prevent some issues...
        robot.arm.reset();
        // Create a drive. Note that passing the entire OpMode is not ideal, should be fixed later
        Drive2 drive = new Drive2(robot, this);
        //Drive3 drive = new Drive3(this);
        //drive.init();

        // Open the camera; also begins streaming the pipeline
        webcam.open();
        cap.setPosition(1);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);

        // Wait for the OpMode to start
        // Make sure to do this after the camera is opened; otherwise "View Camera Stream" won't work
        waitForStart();

        while (!detector.isReady()) {
            sleep(50);
        }

        // As soon as we start, get the position
        TeamElementDetector.TeamElementPosition position = detector.getAnalysis();
        // Print it to the telemetry
        telemetry.log().add("Position of the Team Element: " + position);
        telemetry.update();

        // Drive away from wall so the arm doesn't hit it.
        drive.navigationMonitorTicks(10, 0, -15, 10);
        drive.ceaseMotion();
        cap.setPosition(.5);

        // Raise the arm so it doesn't drag.
        robot.arm.setPositions(-1435, .38);
        drive.navigationMonitorTicks(10, -8, 0, 10);
        sleep(1000);
        //drive.navigationMonitorTicks(10, 2, 15, 10, true);
        drive.navigationLocalizeCarousel(10, -2, 15, 10);
        drive.ceaseMotion();
        drive.navigationMonitorTicks(10, 0, 0.1, 10);
        //drive.navigationMonitorTicks(.1, 0, .5,10, true);
        drive.ceaseMotion();
        leftCarousel.setPower(0.4);
        sleep(3000);
        leftCarousel.setPower(0);


        drive.navigationMonitorTicks(20, -1, -29.5, 10);

        // Rotate 90deg (no movement)
        drive.navigationMonitorTicksPhi(0, -10, 10, 92, 2.5);
        drive.ceaseMotion();

        switch (position) {
            case LEFT:
                robot.arm.goToBackPosition(Arm.HubPosition.BOTTOM);
                sleep(3000);
                drive.navigationMonitorTicksPhi(10, 0, -18, 92, 10);
                drive.ceaseMotion();
                robot.arm.setCollectorMode(Arm.CollectorMode.Eject);
                sleep(3000);
                robot.arm.setCollectorMode(Arm.CollectorMode.Stop);
                // 20 total
                drive.navigationMonitorTicksPhi(10, 0, 6, 92, 10);
                robot.arm.goToBackPosition(Arm.HubPosition.PARK);
                drive.navigationMonitorTicksPhi(10, 0, 14, 92, 10);
                drive.navigationMonitorTicksPhi(10, 16, 0, 92, 10);
                drive.ceaseMotion();
                break;
            case CENTER:
                robot.arm.goToBackPosition(Arm.HubPosition.MIDDLE);
                sleep(3000);
                drive.navigationMonitorTicksPhi(10, 0, -19, 92, 10);
                drive.ceaseMotion();
                robot.arm.setCollectorMode(Arm.CollectorMode.Eject);
                sleep(3000);
                robot.arm.setCollectorMode(Arm.CollectorMode.Stop);

                drive.navigationMonitorTicksPhi(10, 0, 6, 92, 10);
                robot.arm.goToBackPosition(Arm.HubPosition.PARK);
                drive.navigationMonitorTicksPhi(10, 0, 14, 92, 10);
                drive.navigationMonitorTicksPhi(10, 17, 0, 92, 10);
                drive.ceaseMotion();
                break;
            case RIGHT:
                robot.arm.goToBackPosition(Arm.HubPosition.TOP);
                sleep(3000);
                drive.navigationMonitorTicksPhi(10, 0, -21, 92, 10);
                drive.ceaseMotion();
                robot.arm.setCollectorMode(Arm.CollectorMode.Eject);
                sleep(3000);
                robot.arm.setCollectorMode(Arm.CollectorMode.Stop);

                drive.navigationMonitorTicksPhi(10, 0, 6, 92, 10);
                robot.arm.goToBackPosition(Arm.HubPosition.PARK);
                drive.navigationMonitorTicksPhi(10, 0, 16, 92, 10);
                drive.navigationMonitorTicksPhi(10, 16, 0, 92, 10);
                drive.ceaseMotion();
                break;
        }

        drive.navigationMonitorTicksPhi(10, 0, 4, 92, 10);
        drive.ceaseMotion();

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        while (opModeIsActive()) {
            sleep(50);
        }
    }
}

    }
}

