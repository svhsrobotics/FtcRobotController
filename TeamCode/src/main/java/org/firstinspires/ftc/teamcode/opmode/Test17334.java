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
public class Test17334 extends LinearOpMode {





    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private Servo servo;
    private DcMotor low_lift;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "front_right");
        leftBackMotor = hardwareMap.get(DcMotor.class, "back_left");
        rightBackMotor = hardwareMap.get(DcMotor.class, "back_right");
        servo = hardwareMap.get(Servo.class, "pinch");
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
       while(opModeIsActive()){
           double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
           double x = gamepad1.left_stick_x;
           double rx = gamepad1.right_stick_x;

           leftFrontMotor.setPower(y + x + rx);
           leftBackMotor.setPower(y - x + rx);
           rightFrontMotor.setPower(y - x - rx);
           rightBackMotor.setPower(y + x - rx);
           if(gamepad1.right_bumper){
               servo.setPosition(1);

           }
           else if (gamepad1.left_bumper){
               servo.setPosition(-1);
           }
           else if (!(gamepad1.left_bumper && gamepad1.right_bumper)){
               servo.setPosition(0);
           }
           if(gamepad1.a){
               low_lift = hardwareMap.get(DcMotor.class, "lift_low");
               low_lift.setPower(1);
           }

       }
    }

}

