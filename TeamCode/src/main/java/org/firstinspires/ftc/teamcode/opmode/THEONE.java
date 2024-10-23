package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class THEONE extends LinearOpMode {


    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor Arm;
    double reference;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "front_right");
        leftBackMotor = hardwareMap.get(DcMotor.class, "back_left");
        rightBackMotor = hardwareMap.get(DcMotor.class, "back_right");
        Arm = hardwareMap.get(DcMotor.class, "arm");
        double encoderPosition;
        waitForStart();
        double error;
        error = 1.1;
        double power = 0;


        reference = 300;
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {

            while (gamepad1.b) {


                // obtain the encoder position
                encoderPosition = Arm.getCurrentPosition();
                // calculate the error
                error = reference - encoderPosition;
                power = (error/250)*.7;
                if(power>0.7){
                    power = 0.7;
                }
                else if(power<-0.7){
                    power = -0.7;
                }

               Arm.setPower(power);
                while (gamepad1.a) {
                    reference = encoderPosition + 5;
                }
                telemetry.addData("arm", encoderPosition);
                telemetry.addData("error", error);
                telemetry.addData("stick", gamepad2.left_stick_y);
                telemetry.addData("power", power);
                telemetry.update();


            }
            Arm.setPower(0);
            leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            leftFrontMotor.setPower(y + x + rx);
            leftBackMotor.setPower(y - x + rx);
            rightFrontMotor.setPower(y - x - rx);
            rightBackMotor.setPower(y + x - rx);

//oops i didn't push right :P
        }
    }
}


//3 hours for 1 bracket... always remember to click code->reformat code



        





