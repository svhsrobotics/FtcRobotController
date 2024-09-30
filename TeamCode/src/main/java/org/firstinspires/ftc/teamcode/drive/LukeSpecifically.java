package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LukeSpecifically extends LinearOpMode {


    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;



    @Override
    public void runOpMode() throws InterruptedException{
        //the code doesn't wait for start
        front_left = hardwareMap.get(DcMotor.class,"left_front_left_dw");
        front_right = hardwareMap.get(DcMotor.class,"right_front");
        back_left = hardwareMap.get(DcMotor.class,"left_back");
        back_right = hardwareMap.get(DcMotor.class,"right_back_right_dw");
        if(gamepad1.left_stick_y > 0){
            front_left.setPower (1);
            front_right.setPower((1));
            back_left.setPower(1);
            back_right.setPower(1);

        }

    }
}
