package org.firstinspires.ftc.teamcode.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@Config
@TeleOp
public class NickTeleOp extends LinearOpMode {


    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private CRServo Grabber;
    private CRServo tail;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_left_dw");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_right_dw");
        Grabber = hardwareMap.get(CRServo.class, "Grabber");
        tail = hardwareMap.get(CRServo.class, "servo");


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

            if (gamepad1.left_bumper) {
                tail.setPower(1.0);
            } else if (gamepad1.right_bumper) {
                tail.setPower(-1.0);
            } else {
                tail.setPower(0.0);

                if (gamepad1.left_trigger > 0.5) {
                    Grabber.setPower(1.0);
                } else if (gamepad1.right_trigger > 0.5) {
                    Grabber.setPower(-1.0);
                } else {
                    Grabber.setPower(0.0);
                }
            }
//            if (gamepad1.left_stick_y >.5 ) {
//                leftFrontMotor.setPower(1.0);
//                leftBackMotor.setPower(1.0);
//                rightBackMotor.setPower(1.0);
//                rightFrontMotor.setPower(1.0);
//
//
//
//            } else if  (gamepad1.left_stick_y <-.5 ) {
//                leftFrontMotor.setPower(-1.0);
//                leftBackMotor.setPower(-1.0);
//                rightBackMotor.setPower(-1.0);
//                rightFrontMotor.setPower(-1.0);
//
//
//
//            } else {leftFrontMotor.setPower(0);
//                leftBackMotor.setPower(0);
//                rightBackMotor.setPower(0);
//                rightFrontMotor.setPower(0);}
//

        }
    }
}