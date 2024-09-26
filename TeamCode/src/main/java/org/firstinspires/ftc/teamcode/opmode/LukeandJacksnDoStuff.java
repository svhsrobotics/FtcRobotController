package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LukeandJacksnDoStuff extends LinearOpMode {
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()) {

            front_left = hardwareMap.get(DcMotor.class, "front_left");
            front_right = hardwareMap.get(DcMotor.class, "front_right");
            back_right = hardwareMap.get(DcMotor.class, "back_right");
            back_left = hardwareMap.get(DcMotor.class, "back_left");
            if (gamepad1.left_stick_y > 0){
                front_left.setPower(1);
                front_right.setPower(1);
                back_left.setPower(1);
                back_right.setPower(1);
            }
            else{
                //front
                }
            }
        }
    }
}
