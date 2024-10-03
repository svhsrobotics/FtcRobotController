package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class LukeSpecifically extends LinearOpMode {


    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private CRServo servo_dog;
    private CRServo grabber_dog;


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        front_left = hardwareMap.get(DcMotor.class, "left_front_left_dw");
        front_right = hardwareMap.get(DcMotor.class, "right_front");
        back_left = hardwareMap.get(DcMotor.class, "left_back");
        back_right = hardwareMap.get(DcMotor.class, "right_back_right_dw");
        servo_dog = hardwareMap.get(CRServo.class, "servo");
        grabber_dog = hardwareMap.get(CRServo.class, "Grabber");
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            front_left.setPower(-(y + x + rx));
            back_left.setPower(-(y - x + rx));
            front_right.setPower(y - x - rx);
            back_right.setPower(y + x - rx);

            if (gamepad1.right_bumper) {
                servo_dog.setPower(1);
            }//use if else and else statements before i kill you
            if (gamepad1.left_bumper) {
                servo_dog.setPower(0);
            }
            if (gamepad1.dpad_left) {
                grabber_dog.setPower(-1);
            }//make it stop
            if (gamepad1.dpad_right) {
                grabber_dog.setPower(1);
            }
        }


    }

}
