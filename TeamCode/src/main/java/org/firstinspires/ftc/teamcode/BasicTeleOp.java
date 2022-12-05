package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBot;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.Drive;
import org.firstinspires.ftc.teamcode.util.ExMath;
import org.firstinspires.ftc.teamcode.util.Logger;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class BasicTeleOp extends LinearOpMode {
    Logger logger = new Logger(telemetry);

    double wrist = 1;
    int slide = 0;
    int pitch = 0;

    @Override
    public void runOpMode() {
        PowerPlayBot robot = new PowerPlayBot(hardwareMap, logger);
        robot.initHardware();

        // TODO: Move this to the robot class
        Servo flipper = hardwareMap.get(Servo.class, "flip");

        //robot.grabber.lift.reset(); // Uncomment to reset the encoders on startup

        robot.grabber.pinch.setPinchPosition(0);
        robot.grabber.lift.slide.setDirection(DcMotorSimple.Direction.REVERSE); // Slide is inverted

        waitForStart();

        while(!isStopRequested()) {
            // BEGIN DRIVE
            robot.drives.get(Robot.DrivePos.FRONT_LEFT).setPower(
                    (gamepad1.right_trigger-gamepad1.left_trigger+Math.pow(gamepad1.left_stick_x, 1.5)+gamepad1.right_stick_x)
                    + (gamepad2.right_trigger-gamepad2.left_trigger+Math.pow(gamepad2.left_stick_x, 1.5)+gamepad2.right_stick_x));
            robot.drives.get(Robot.DrivePos.FRONT_RIGHT).setPower(
                    (gamepad1.right_trigger-gamepad1.left_trigger-Math.pow(gamepad1.left_stick_x, 1.5)-gamepad1.right_stick_x)
                    + (gamepad2.right_trigger-gamepad2.left_trigger-Math.pow(gamepad2.left_stick_x, 1.5)-gamepad2.right_stick_x));
            robot.drives.get(Robot.DrivePos.BACK_LEFT).setPower(
                    (gamepad1.right_trigger-gamepad1.left_trigger+Math.pow(gamepad1.left_stick_x, 1.5)-gamepad1.right_stick_x)
                    + (gamepad2.right_trigger-gamepad2.left_trigger+Math.pow(gamepad2.left_stick_x, 1.5)-gamepad2.right_stick_x));
            robot.drives.get(Robot.DrivePos.BACK_RIGHT).setPower(
                    (gamepad1.right_trigger-gamepad1.left_trigger-Math.pow(gamepad1.left_stick_x, 1.5)+gamepad1.right_stick_x)
                    + (gamepad2.right_trigger-gamepad2.left_trigger-Math.pow(gamepad2.left_stick_x, 1.5)+gamepad2.right_stick_x));
            // END DRIVE

            // BEGIN ARM (and misc related controls)


            // Begin Pincher
            if (gamepad1.a || gamepad2.a) {
                robot.grabber.hand.setPinchPosition(0.45);
            } else if (gamepad1.b || gamepad2.b) {
                robot.grabber.hand.setPinchPosition(0);
            }

            // End Pincher

            // Begin Wrist
            /*if (gamepad1.dpad_right || gamepad2.dpad_right) {
                wrist += 0.002;
            } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
                wrist -= 0.002;
            }

            if (wrist > 1) {
                wrist = 1;
            } else if (wrist < 0) {
                wrist = 0;
            }

            telemetry.addData("Wrist", wrist);
            robot.grabber.hand.setWristPosition(wrist);
            // End Wrist */

            // Begin Slide
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                slide -= 10;
            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
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
            // End Slide

            // Begin Pitch
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                pitch += 10;
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                pitch -= 10;
            }

            if (pitch > 5320) {
                //pitch = 5320;
            } else if (pitch < 0) {
                //pitch = 0;
            }

            robot.grabber.lift.pitch.setTargetPosition(pitch);
            robot.grabber.lift.pitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.grabber.lift.pitch.setPower(1);

            telemetry.addData("Pitch", pitch);
            // End Pitch

            // Begin Flipper
            if (gamepad1.x || gamepad2.x) {
                flipper.setPosition(1);
            } else if (gamepad1.y || gamepad2.y) {
                flipper.setPosition(0);
            }
            // END ARM

            telemetry.update(); // Update telemetry

        }

    }
}
