package org.firstinspires.ftc.teamcode.Shared;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.vision.pole.DoubleThresholdPipeline;

public class PoleLocalize {
    /// This is only used in order to call isStopRequested() during the navigationMonitorTicks loop, and for the telemetry
    LinearOpMode opMode;
    Logger logger;
    /// The robot class, contains all the hardware
    Robot robot;

    DoubleThresholdPipeline pipeline;

    double integrationSum = 0;
    double P_GAIN = (0.06 / 8.0) / 2;
    //double I_GAIN = 0.04 / 8.0;
    double I_GAIN = 0;


    public PoleLocalize(Robot robot, LinearOpMode opMode, DoubleThresholdPipeline pipeline) {
        this.robot = robot;
        this.opMode = opMode;
        this.logger = new Logger(opMode.telemetry, true);

        this.pipeline = pipeline;
    }

    public void followThePole() {
        int settleCounter = 0;
        while (settleCounter < 3 && (opMode.opModeIsActive() || opMode.opModeInInit())) {
            if (Math.abs(pipeline.necessaryCorrection()) < 5) {
                // If we're really close to the target, increase the settle counter
                settleCounter++;
            } else {
                // If we're not close, reset the settle counter
                settleCounter = 0;
            }
            adjustPowersFromPID();
        }
        ceaseMotion();
    }

    public void ceaseMotion() {
        for (Drive drive : robot.drives.values()) {
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.setPower(0);
        }
    }

    private double calculatePIDCorrection() {
        double error = -pipeline.necessaryCorrection();

        integrationSum += error;
        // Prevent windup
        integrationSum = clip(integrationSum, -3, 3);

        return (error * P_GAIN) + (integrationSum * I_GAIN);
    }

    @SuppressWarnings("ConstantConditions")
    private void adjustPowersFromPID() {
        logger.info("followPowerCorrection was called");
        double powerCorrection = calculatePIDCorrection();

        // Get the drives and set their powers
        // Left side is negated
        robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(-powerCorrection);
        robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(-powerCorrection);
        robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(powerCorrection);
        robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(powerCorrection);

    }
}