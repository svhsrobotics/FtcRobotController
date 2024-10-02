package org.firstinspires.ftc.teamcode.opmode.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;


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
        float lx;
        float rx;
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
             topLeftMotor.setPower((ly-lx-rx)*motorSpeed);
            // topRightMotor.setPower((ly-lx-rx)*motorSpeed);
          //   bottomLeftMotor.setPower((ly-lx+rx)*motorSpeed);
          //   bottomRightMotor.setPower((-rx+lx+ly)*motorSpeed);
             /*
             if (gamepad1.a) {
                tail.setDirection(DcMotorSimple.Direction.FORWARD);
             }
             else {
                 tail.setDirection(DcMotorSimple.Direction.REVERSE);
             }

              */
//             System.out.println();
        }
    }
}

