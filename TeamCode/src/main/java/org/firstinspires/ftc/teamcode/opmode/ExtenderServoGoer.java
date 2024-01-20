package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp

public class ExtenderServoGoer extends LinearOpMode  {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo extenderServo = hardwareMap.get(CRServo.class, "ExtenderServo");
        AnalogInput extenderAnalog = hardwareMap.get(AnalogInput.class, "ExtenderAnalog");

        waitForStart();
        while (!isStopRequested()) {
            if (gamepad1.right_bumper) {
                extenderServo.setPower(0.1);
            } else if (gamepad1.left_bumper) {
                extenderServo.setPower(-0.1);

            } else {
                extenderServo.setPower(0);
            }
        }
    }

}
