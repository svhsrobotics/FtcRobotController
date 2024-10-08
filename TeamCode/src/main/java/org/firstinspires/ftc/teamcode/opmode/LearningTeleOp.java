package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class LearningTeleOp extends LinearOpMode {


    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private CRServo Grabber;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_left_dw");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_right_dw");
        Grabber = hardwareMap.get(CRServo.class, "Grabber");

        waitForStart();
        while (opModeIsActive()) {
            leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            leftFrontMotor.setPower(y + x + rx);
            leftBackMotor.setPower(y - x + rx);
            rightFrontMotor.setPower(y - x - rx);
            rightBackMotor.setPower(y + x - rx);
            if (gamepad1.a){
                leftBackMotor.setPower(2);
            rightBackMotor.setPower(2);
            leftFrontMotor.setPower(2);
            rightFrontMotor.setPower(2);}



            if (gamepad1.left_bumper)
                Grabber.setPower(1);
            else if (gamepad1.right_bumper)
                Grabber.setPower(-1);
            else Grabber.setPower(0);
            //3 hours for 1 bracket... always remember to click code->reformat code

            //TODO: Mess with servo in slot 2 in order to make the dog's arm rotate

        }

    }
}

