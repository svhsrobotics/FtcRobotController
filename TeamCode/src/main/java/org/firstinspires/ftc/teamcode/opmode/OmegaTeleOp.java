package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class OmegaTeleOp extends LinearOpMode {
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor bottomLeftMotor;
    private DcMotor bottomRightMotor;

    private DcMotor trumansArm = hardwareMap.get(DcMotor.class, "arm");

    @Override

    public void runOpMode() throws InterruptedException {
        topLeftMotor = hardwareMap.get(DcMotor.class,"FL");
        topRightMotor = hardwareMap.get(DcMotor.class,"FR");
        bottomLeftMotor = hardwareMap.get(DcMotor.class,"BL");
        bottomRightMotor = hardwareMap.get(DcMotor.class,"BR");
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx2 = gamepad1.right_stick_x;

            topLeftMotor.setPower(y + x + rx2);
            bottomLeftMotor.setPower(y - x + rx2);
            topRightMotor.setPower(y - x - rx2);
            bottomRightMotor.setPower(y + x - rx2);
        }
    }
}
