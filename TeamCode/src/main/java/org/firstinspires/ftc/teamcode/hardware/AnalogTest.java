package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class AnalogTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //AxonServo servoTest = new AxonServo("servo", "a0", hardwareMap);
        Servo elbowServo = hardwareMap.get(Servo.class, "elbow");

        waitForStart();
        double elbowPos = 0;
        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                servoTest.innerServo.setPower(0.6);
//            } else {
//                servoTest.innerServo.setPower(0);
//            }

            elbowPos += gamepad1.left_stick_x * 0.001;
            elbowServo.setPosition(elbowPos); // 0.2 down 0.55 up

            telemetry.addData("Elbow Pos", elbowPos);

            telemetry.update();
        }
    }
}
