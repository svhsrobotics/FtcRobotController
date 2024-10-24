package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PatrickTeleOP extends LinearOpMode {

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor arm;
    double reference;


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_right");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        arm = hardwareMap.get(DcMotor.class, "arm");
        double encoderPosition;
        waitForStart();
        double error;
        error = 1.1;
        double power = 0;


        while (opModeIsActive()) {
            front_left.setDirection(DcMotorSimple.Direction.REVERSE);
            back_left.setDirection(DcMotorSimple.Direction.REVERSE);
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            front_left.setPower(y + x + rx);
            back_left.setPower(y - x + rx);
            front_right.setPower(y - x - rx);
            back_right.setPower(y + x - rx);
            if (gamepad1.left_bumper) {
                front_left.setPower(1);
                back_left.setPower(1);
            } else {
                front_left.setPower(0);
                back_left.setPower(0);
            }
            if (gamepad1.right_bumper) {
                front_right.setPower(1);
                back_right.setPower(1);
            } else {
                front_right.setPower(0);
                back_right.setPower(0);
            }
            reference = 300;
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (gamepad1.b) {

                encoderPosition = arm.getCurrentPosition();
                error = reference - encoderPosition;
                power = (error / 250) * .7;
                if (power > 0.7) {
                    power = 0.7;
                } else if (power < -0.7) {
                    power = -0.7;
                }

                arm.setPower(power);

                telemetry.addData("arm", encoderPosition);
                telemetry.addData("error", error);
                telemetry.addData("stick", gamepad2.left_stick_y);
                telemetry.addData("power", power);
                telemetry.update();
            }
        }
    }
}

