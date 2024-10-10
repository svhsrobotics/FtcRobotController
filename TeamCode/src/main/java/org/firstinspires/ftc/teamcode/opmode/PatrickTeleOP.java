package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class PatrickTeleOP extends LinearOpMode {

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        front_left = hardwareMap.get(DcMotor.class, "left_front_left_dw");
        front_right = hardwareMap.get(DcMotor.class, "right_front");
        back_left = hardwareMap.get(DcMotor.class, "left_back");
        back_right = hardwareMap.get(DcMotor.class, "right_back_right_dw");

        while (opModeIsActive()) {
            if (gamepad1.left_stick_x > 0) {
            }
        }

    }


}

