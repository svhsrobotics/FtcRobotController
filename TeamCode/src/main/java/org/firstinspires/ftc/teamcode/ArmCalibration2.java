package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.CompetitionRobot;
import org.firstinspires.ftc.teamcode.robot.PowerPlayBot2;
import org.firstinspires.ftc.teamcode.robot.hardware.Grabber2;
import org.firstinspires.ftc.teamcode.robot.hardware.Hand;
import org.firstinspires.ftc.teamcode.util.Logger;

@Disabled
@TeleOp
public class ArmCalibration2 extends LinearOpMode {
    Logger logger = new Logger(telemetry);
    int y = 1;
    int x = 0;

    @Override
    public void runOpMode() {
        PowerPlayBot2 robot = new PowerPlayBot2(hardwareMap, logger);
        robot.initHardware();
        //robot.grabber.lift.reset();
        waitForStart();

        while(!isStopRequested()) {
            if (gamepad2.dpad_up) {
                y += 0.002;
            } else if (gamepad2.dpad_down) {
                y -= 0.002;
            }

            if (gamepad2.dpad_left) {
                x -= 10;
            } else if (gamepad2.dpad_right) {
                x += 10;
            }


            // Limit the slide to range 0 to 5600
            if (x > 5690) {
                x = 5690;
            } else if (x < 0) {
                x = 0;
            }

            robot.grabber2.slides.setSlidePositions(y, x);
            robot.grabber2.slides.topmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.grabber2.slides.topmotor.setPower(1);
            robot.grabber2.slides.middlemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.grabber2.slides.middlemotor.setPower(1);
            robot.grabber2.slides.bottommotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.grabber2.slides.bottommotor.setPower(1);
            robot.grabber2.slides.hslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.grabber2.slides.hslide.setPower(0);

            telemetry.addData("LiftSlide", y);


            telemetry.update();

        }

    }
}
