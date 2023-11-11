package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class AnalogTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AxonServo servoTest = new AxonServo("servo test", "servoadc", hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a){
                servoTest.innerServo.setPosition(0.6);}
             else
                {servoTest.innerServo.setPosition(0.5);}
            servoTest.innerAnalog.getVoltage();
             servoTest.getCurrentPosition();
             telemetry.addData("Degrees",servoTest.getCurrentPosition());
            telemetry.addData("Voltage", servoTest.innerAnalog.getVoltage());

            telemetry.update();
        }
    }
}
