package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.PowerPlayBot;
import org.firstinspires.ftc.teamcode.util.Logger;

@Disabled
@TeleOp
public class ArmCalibration extends LinearOpMode {
    Logger logger = new Logger(telemetry);
    double wrist = 1;
    int slide = 0;
    boolean pinch = false;
    int pitch = 0;

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
            if (gamepad1.a) {
                pinch = true;
                robot.grabber.hand.setPinchPosition(0.45);
            } else if (gamepad1.b) {
                pinch = false;
                robot.grabber.hand.setPinchPosition(0);
            }
            telemetry.addData("Pinch", pinch);

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

            if (gamepad1.dpad_left) {
                slide -= 10;
            } else if (gamepad1.dpad_right) {
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
                    robot.grabber.lift.pitch.setPower(Math.pow(gamepad1.right_trigger, 2));
                /*} else {
                    robot.grabber.lift.pitch.setPower(0);
                }*/
            } else if (gamepad1.left_trigger > 0.2) {
                //if (robot.grabber.lift.pitch.getCurrentPosition() > 400) {
                    robot.grabber.lift.pitch.setPower(-Math.pow(gamepad1.left_trigger, 2));
                /*} else {
                    robot.grabber.lift.pitch.setPower(0);
                }*/
            } else {
                robot.grabber.lift.pitch.setPower(0);
            }

            telemetry.addData("Pitch", robot.grabber.lift.pitch.getCurrentPosition());

            telemetry.update();

        }

    }
}
