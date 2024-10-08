package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ActualGavinTestTeleOp extends LinearOpMode {
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;


    public void runOpMode() throws InterruptedException {

        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");

        waitForStart();
        while(opModeIsActive())
            if (gamepad1.right_stick_x>=.3 && gamepad1.left_stick_y>=.3){
                front_left.setPower(1);
                front_right.setPower(0);
                back_right.setPower(0);
                back_left.setPower(1);
            }
            else if (gamepad1.right_stick_y>=0.3) {
                front_left.setPower(1);
                front_right.setPower(-1);
                back_right.setPower(-1);
                back_left.setPower(1);


            } else if (gamepad1.right_stick_y<=-0.3) {
                front_left.setPower(-1);
                front_right.setPower(1);
                back_right.setPower(1);
                back_left.setPower(-1);


            }
            else if (gamepad1.left_stick_x<=-.3) {
                front_left.setPower(1);
                front_right.setPower(1);
                back_left.setPower(1);
                back_right.setPower(1);
            } else if (gamepad1.left_stick_x>=.3) {
                front_left.setPower(-1);
                front_right.setPower(-1);
                back_left.setPower(-1);
                back_right.setPower(-1);

            } else {
                front_right.setPower(0);
                front_left.setPower(0);
                back_left.setPower(0);
                back_right.setPower(0);
            }



    }}
