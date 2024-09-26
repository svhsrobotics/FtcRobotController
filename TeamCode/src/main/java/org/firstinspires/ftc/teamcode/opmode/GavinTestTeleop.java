package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class GavinTestTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_left_dw");
        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        DcMotor leftBackMotor = hardwareMap.get (DcMotor.class, "left_back");
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor rightBackMotor = hardwareMap.get (DcMotor.class, "right_back_right_dw");

        waitForStart();
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            leftFrontMotor.setPower(y + x + rx);
            leftBackMotor.setPower(y - x + rx);
            rightFrontMotor.setPower(y - x - rx);
            rightBackMotor.setPower(y + x - rx);
        }
    }
}
