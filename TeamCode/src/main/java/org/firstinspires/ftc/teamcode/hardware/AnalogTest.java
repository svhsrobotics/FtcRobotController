package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
//@Disabled
public class AnalogTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AxonServo servoTest = new AxonServo("servo", "a0", hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a){
                servoTest.innerServo.setPower(0.6);}
             else
                {servoTest.innerServo.setPower(0.2);}
            servoTest.innerAnalog.getVoltage();
             servoTest.getCurrentPosition();
             telemetry.addData("Degrees",servoTest.getCurrentPosition());
            telemetry.addData("Voltage", servoTest.innerAnalog.getVoltage());

            telemetry.update();
        }
    }
}
