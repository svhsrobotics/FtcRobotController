package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class nickpidloop extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor Arm;
    double refrence;

    public void runOpMode()  throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "front_right");
        leftBackMotor = hardwareMap.get(DcMotor.class, "back_left");
        rightBackMotor = hardwareMap.get(DcMotor.class, "back_right");
        Arm = hardwareMap.get(DcMotor.class, "arm");
        double encoderPosition = 0;
        waitForStart();
        double error = 1.1;
        double power = 0;
        double difference = 0;
        double preverror = 0;


        refrence = 300;
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {

            while (gamepad1.b);

                error = preverror + error;
                difference = error -preverror;
                preverror = error;
//                power = error + what?? ask brennan on monday

                }

        }
    }


