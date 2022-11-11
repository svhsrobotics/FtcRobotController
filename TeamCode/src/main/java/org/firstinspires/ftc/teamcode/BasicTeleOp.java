package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBot;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.util.ExMath;
import org.firstinspires.ftc.teamcode.util.Logger;

@TeleOp
public class BasicTeleOp extends LinearOpMode {
    Logger logger = new Logger(telemetry);
    double wrist = 1;
    int slide = 0;
    int pitch = 0;
    Telemetry.Line errorLine;

    @Override
    public void runOpMode() {
        PowerPlayBot robot = new PowerPlayBot(hardwareMap, logger);
        robot.initHardware();
        //robot.grabber.lift.reset();
        robot.grabber.hand.setPinchPosition(0);
        robot.grabber.lift.slide.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.grabber.lift.pitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while(!isStopRequested()) {
            // DRIVE CODE
            Drive leftFrontDrive = robot.drives.get(Robot.DrivePos.FRONT_LEFT);
            Drive rightFrontDrive = robot.drives.get(Robot.DrivePos.FRONT_RIGHT);
            Drive leftBackDrive = robot.drives.get(Robot.DrivePos.BACK_LEFT);
            Drive rightBackDrive = robot.drives.get(Robot.DrivePos.BACK_RIGHT);

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            double frontRightPowerFactor, frontLeftPowerFactor, backRightPowerFactor, backLeftPowerFactor;
            double magRight = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double thetaRight = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
            double magLeft = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double thetaLeft = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            double pi = Math.PI;
            int leftSign = 1;
            int rightSign = 1;

            if (gamepad1.left_bumper) { // Changed from left trigger as it's used for arm
                magRight = ExMath.square_with_sign(magRight) / 2;
                magLeft = ExMath.square_with_sign(magLeft) / 2;
            }

            if (thetaRight > 0 && thetaRight < pi / 2) {
                frontRightPowerFactor = -Math.cos(2 * thetaRight);
            } else if (thetaRight >= -pi && thetaRight < -pi / 2) {
                frontRightPowerFactor = Math.cos(2 * thetaRight);
            } else if (thetaRight >= pi / 2 && thetaRight <= pi) {
                frontRightPowerFactor = 1;
            } else {
                frontRightPowerFactor = -1;
            }

            if (thetaLeft > 0 && thetaLeft < pi / 2) {
                backLeftPowerFactor = -Math.cos(2 * thetaLeft);
            } else if (thetaLeft >= -pi && thetaLeft < -pi / 2) {
                backLeftPowerFactor = Math.cos(2 * thetaLeft);
            } else if (thetaLeft >= pi / 2 && thetaLeft <= pi) {
                backLeftPowerFactor = 1;
            } else {
                backLeftPowerFactor = -1;
            }

            if (thetaRight > -pi / 2 && thetaRight < 0) {
                backRightPowerFactor = Math.cos(2 * thetaRight);
            } else if (thetaRight > pi / 2 && thetaRight < pi) {
                backRightPowerFactor = -Math.cos(2 * thetaRight);
            } else if (thetaRight >= 0 && thetaRight <= pi / 2) {
                backRightPowerFactor = 1;
            } else {
                backRightPowerFactor = -1;
            }

            if (thetaLeft > -pi / 2 && thetaLeft < 0) {
                frontLeftPowerFactor = Math.cos(2 * thetaLeft);
            } else if (thetaLeft > pi / 2 && thetaLeft < pi) {
                frontLeftPowerFactor = -Math.cos(2 * thetaLeft);
            } else if (thetaLeft >= 0 && thetaLeft <= pi / 2) {
                frontLeftPowerFactor = 1;
            } else {
                frontLeftPowerFactor = -1;
            }

            // Redundant assignment removed, defaults to positive
            if (frontLeftPowerFactor < 0){
                leftSign=-1;
            } else if (frontRightPowerFactor<0) {
                rightSign=-1;
            }
            // leftFrontDrive.setPower((frontLeftPowerFactor * magLeft)*(frontLeftPowerFactor * magLeft));
            // rightFrontDrive.setPower(-(frontRightPowerFactor * magRight)*(frontRightPowerFactor * magRight));
            //leftBackDrive.setPower((backLeftPowerFactor * magLeft)*(backLeftPowerFactor * magLeft));
            //rightBackDrive.setPower(-(backRightPowerFactor * magRight)*(backRightPowerFactor * magRight));

            leftFrontDrive.setPower(((ExMath.square_with_sign(frontLeftPowerFactor) * magLeft)));
            rightFrontDrive.setPower((ExMath.square_with_sign(frontRightPowerFactor) * magRight));
            leftBackDrive.setPower(((ExMath.square_with_sign(backLeftPowerFactor) * magLeft)));
            rightBackDrive.setPower((ExMath.square_with_sign(backRightPowerFactor) * magRight));
            
            // ARM CODE

            if (gamepad1.a) {
                robot.grabber.hand.setPinchPosition(0.45);
            } else if (gamepad1.b) {
                robot.grabber.hand.setPinchPosition(0);
            }

            if (gamepad1.dpad_up) {
                wrist += 0.002;
            } else if (gamepad1.dpad_down) {
                wrist -= 0.002;
            }

            if (wrist > 1) {
                wrist = 1;
            } else if (wrist < 0) {
                wrist = 0;
            }

            telemetry.addData("Wrist", wrist);
            robot.grabber.hand.setWristPosition(wrist);

            if (gamepad1.left_bumper) {
                slide -= 10;
            } else if (gamepad1.right_bumper) {
                slide += 10;
            }


            // Limit the slide to range 0 to 5600
            if (slide > 5690) {
                slide = 5690;
            } else if (slide < 0) {
                slide = 0;
            }

            robot.grabber.lift.slide.setTargetPosition(slide);
            robot.grabber.lift.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.grabber.lift.slide.setPower(1);

            telemetry.addData("Slide", slide);


            if (gamepad1.right_trigger > 0.2) {
                //if (robot.grabber.lift.pitch.getCurrentPosition() < 5320) {
                if (robot.grabber.lift.pitch.getCurrent(CurrentUnit.AMPS) < 4.5) {
                    telemetry.removeLine(errorLine);
                    robot.grabber.lift.pitch.setPower(Math.pow(gamepad1.right_trigger, 2));
                } else {
                    //logger.error("Pitch motor current too high");
                    errorLine = telemetry.addLine(Logger.formatColor("Pitch motor current too high", "#F44336"));
                    robot.grabber.lift.pitch.setPower(0);
                }
            } else if (gamepad1.left_trigger > 0.2) {
                //if (robot.grabber.lift.pitch.getCurrentPosition() > 400) {
                if (robot.grabber.lift.pitch.getCurrent(CurrentUnit.AMPS) < 4.5) {
                    telemetry.removeLine(errorLine);
                    robot.grabber.lift.pitch.setPower(-Math.pow(gamepad1.left_trigger, 2));
                } else {
                    //logger.error("Pitch motor current too high");
                    errorLine = telemetry.addLine(Logger.formatColor("Pitch motor current too high", "#F44336"));
                    robot.grabber.lift.pitch.setPower(0);
                }
            } else {
                robot.grabber.lift.pitch.setPower(0);
            }

            telemetry.addData("Pitch", robot.grabber.lift.pitch.getCurrentPosition());

            telemetry.update();

        }

    }
}
