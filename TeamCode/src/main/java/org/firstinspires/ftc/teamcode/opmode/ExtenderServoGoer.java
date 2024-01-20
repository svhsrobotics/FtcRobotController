package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.AxonServo;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;

@TeleOp

public class ExtenderServoGoer extends LinearOpMode  {
    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        CRServo extenderServo = hardwareMap.get(CRServo.class, "ExtenderServo");
        AnalogInput extenderAnalog = hardwareMap.get(AnalogInput.class, "ExtenderAnalog");
        AxonServo extenderAxonServo = new AxonServo("ExtenderServo", "ExtenderAnalog", hardwareMap);

        waitForStart();
        while (!isStopRequested()) {
            if (gamepad1.right_bumper) {
                extenderServo.setPower(0.1);
            } else if (gamepad1.left_bumper) {
                extenderServo.setPower(-0.1);

            } else {
                extenderServo.setPower(0);
            }
            if (gamepad1.a) {
                extenderAxonServo.setAdjustedPosition(-1130, 0.1);
            } else if (gamepad1.b) {
                extenderAxonServo.setAdjustedPosition(300, 0.1);
            }
//            telemetry.addData("Servo Extender Analog Power", extenderAnalog.getVoltage());
//            telemetry.update();
            telemetry.addData("Extender Axon Servo Position", extenderAxonServo.getAdjustedPosition());
            telemetry.update();
            //max extension: -1150 degrees
            //min position 300 degrees

        }
    }

}
