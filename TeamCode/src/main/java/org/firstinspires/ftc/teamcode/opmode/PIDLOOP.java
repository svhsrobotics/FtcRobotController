package org.firstinspires.ftc.teamcode.opmode;
////this loop is for driving forward and backwards////

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
@Config
public class PIDLOOP extends LinearOpMode {
    private double k1 = 0.01;
    private double k2 = -0.0002;
    private double k3 = 0.0;
    private double reference = 300;
    private double ierror = 0;
    private double diff = 0;
    private double prevError = 0;
    public void runOpMode() throws InterruptedException {
        DcMotor topLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        DcMotor topRightMotor = hardwareMap.get(DcMotor.class, "FR");
        DcMotor bottomLeftMotor = hardwareMap.get(DcMotor.class, "BL");
        DcMotor bottomRightMotor = hardwareMap.get(DcMotor.class, "BR");

        topLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("ticks",topLeftMotor.getCurrentPosition());
            double error = reference - topLeftMotor.getCurrentPosition();
            ierror = ierror + error;
            diff = error - prevError;
            prevError = error;
            topLeftMotor.setPower(k1*error+k2*ierror+k3*diff);
            telemetry.update();
        }
    }
}
