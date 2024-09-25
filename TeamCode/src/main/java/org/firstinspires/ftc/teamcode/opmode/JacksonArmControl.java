package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class JacksonArmControl extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor shoulderMotor = hardwareMap.get(DcMotorEx.class, "shoulder");
        CRServo elbowServo = hardwareMap.get(CRServo.class, "elbow");
        waitForStart();
        while (opModeIsActive()) {
            shoulderMotor.setPower(gamepad1.left_stick_y);
            elbowServo.setPower(gamepad1.right_stick_y);
        }
    }
}
