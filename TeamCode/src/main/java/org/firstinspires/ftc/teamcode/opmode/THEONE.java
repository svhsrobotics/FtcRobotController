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

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "front_right");
        leftBackMotor = hardwareMap.get(DcMotor.class, "back_left");
        rightBackMotor = hardwareMap.get(DcMotor.class, "back_right");
        Arm = hardwareMap.get(DcMotor.class, "arm");

        waitForStart();
        while (opModeIsActive()) {
            leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            leftFrontMotor.setPower(y + x + rx);
            leftBackMotor.setPower(y - x + rx);
            rightFrontMotor.setPower(y - x - rx);
            rightBackMotor.setPower(y + x - rx);
            if (gamepad1.a) {
                leftBackMotor.setPower(2);
                rightBackMotor.setPower(2);
                leftFrontMotor.setPower(2);
                rightFrontMotor.setPower(2);
            }


            if (gamepad1.left_bumper) {
                Arm.setPower(.3);
            } else if (gamepad1.right_bumper) {
                Arm.setPower(-.3);
            } else {
                /*current_time = get_current_time()
                current_error = desire_position-current_position

                p = k_p * current_error

                i += k_i * (current_error * (current_time - previous_time))

                if i > max_i:
                i = max_i
                elif i < -max_i:
                i = -max_i

                D = k_d * (current_error - previous_error) / (current_time - previous_time)

                output = p + i + d

                previous_error = current_error
                previous_time = current_time
                Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Arm.setPower(0);
            }
            //3 hours for 1 bracket... always remember to click code->reformat code


        }

    }
}

