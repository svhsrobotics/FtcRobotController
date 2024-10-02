package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config

@TeleOp
public class LearningTeleOpfor17334 extends LinearOpMode {
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_left_dw");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        waitForStart();
       while(opModeIsActive()){
           if (gamepad1.right_stick_y>=0.1) {
                leftFrontMotor.setPower(1);
                rightFrontMotor.setPower(-1);
              } else if (gamepad1.right_stick_y<=-0.1) {
                leftFrontMotor.setPower(-1);
                rightFrontMotor.setPower(1);
              } else {
                leftFrontMotor.setPower(0);
                rightFrontMotor.setPower(0);

           }


       }




    }
}
