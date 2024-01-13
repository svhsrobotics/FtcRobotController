package org.firstinspires.ftc.teamcode.opmode.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robot;

@TeleOp

public class ButtonConfig extends LinearOpMode {

    private boolean dropPurplePixel = false;
    private boolean parkInner = false;
    private boolean parkOuter = false;
    private boolean placePixel = false;




    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.a) {
                if (!dropPurplePixel) {
                    telemetry.addData("Config: ", "Robot will drop Purple Pixel.");
                    dropPurplePixel = true;
                } else {
                    telemetry.addData("Config: ", "Robot will not drop Purple Pixel.");
                    dropPurplePixel = false;
                }
            }
            if(gamepad1.b) {
                if (!parkInner) {
                    telemetry.addData("Config: ", "Robot will park on the inner spot.");
                    parkInner = true;
                    parkOuter = false;
                } else {
                    telemetry.addData("Config: ", "Robot will not park on inner spot.");
                    parkInner = false;
                }
            }
            if (gamepad1.y) {
                if (!parkOuter) {
                    telemetry.addData("Config: ", "Robot will park on the outer spot.");
                    parkOuter = true;
                    parkInner = false;
                } else {
                    telemetry.addData("Config: ", "Robot will not park on outer spot.");
                    parkOuter = false;
                }
            }
            if (gamepad1.x) {
                if (!placePixel) {
                    telemetry.addData("Config: ", "Robot will place pixel.");
                    placePixel = true;

                } else {
                    telemetry.addData("Config: ", "Robot will not place pixel.");
                    placePixel = false;
                }
            }

        }


    }


}
