package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp

public class TrumanLearnsTeleOP extends LinearOpMode {
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor bottomLeftMotor;
    private DcMotor bottomRightMotor;
    private CRServo tail;
    @Override
    public void runOpMode() throws InterruptedException {
        float ly;
        float MaxSpeed= 0.8F;
        float lx;
        float rx;
        float ry;
        topLeftMotor = hardwareMap.get(DcMotor.class,"FL");
        topRightMotor = hardwareMap.get(DcMotor.class,"FR");
        bottomLeftMotor = hardwareMap.get(DcMotor.class,"BL");
        bottomRightMotor = hardwareMap.get(DcMotor.class,"BR");
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //tail = hardwareMap.get(CRServo.class,"servo"); ONLY FOR DOG
        int tailAngle = 1;
        double motorSpeed = 0.3;
        waitForStart();
        while (opModeIsActive()){
             lx=gamepad1.left_stick_x;
             ly=gamepad1.left_stick_y;
             rx=gamepad1.right_stick_x;
             ry=gamepad1.right_stick_y;

             lx= (float) (lx*MaxSpeed);
           if(gamepad1.left_bumper){
               MaxSpeed= (float) (MaxSpeed+0.1);

           }
            if (gamepad1.right_bumper){
                MaxSpeed= (float) (MaxSpeed-0.1);
            }

            topLeftMotor.setPower(lx);
            bottomLeftMotor.setPower(lx);
             topRightMotor.setPower(lx);
             bottomRightMotor.setPower(lx);

        }
    }
}

