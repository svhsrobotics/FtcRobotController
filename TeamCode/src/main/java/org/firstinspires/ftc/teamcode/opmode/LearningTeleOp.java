package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class LearningTeleOp extends LinearOpMode {





    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_left_dw");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_right_dw");

        waitForStart();
       while(opModeIsActive()){
           if (gamepad1.right_stick_x>=.3 && gamepad1.left_stick_y>=.3){
               leftFrontMotor.setPower(1);
               rightFrontMotor.setPower(0);
               rightBackMotor.setPower(0);
               leftBackMotor.setPower(1);
           }
           else if (gamepad1.right_stick_y>=0.3) {
                leftFrontMotor.setPower(1);
                rightFrontMotor.setPower(-1);
                rightBackMotor.setPower(-1);
                leftBackMotor.setPower(1);


              } else if (gamepad1.right_stick_y<=-0.3) {
                leftFrontMotor.setPower(-1);
                rightFrontMotor.setPower(1);
               rightBackMotor.setPower(1);
               leftBackMotor.setPower(-1);


           }
           else if (gamepad1.left_stick_x<=-.3) {
               leftFrontMotor.setPower(1);
               rightFrontMotor.setPower(1);
               leftBackMotor.setPower(1);
               rightBackMotor.setPower(1);
           } else if (gamepad1.left_stick_x>=.3) {
               leftFrontMotor.setPower(-1);
               rightFrontMotor.setPower(-1);
               leftBackMotor.setPower(-1);
               rightBackMotor.setPower(-1);

           } else {
               rightFrontMotor.setPower(0);
               leftFrontMotor.setPower(0);
               leftBackMotor.setPower(0);
               rightBackMotor.setPower(0);
           }



       }




    }
}
