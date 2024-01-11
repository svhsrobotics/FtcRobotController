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
//                android.util.Log.i("CONFIG", "Robot will drop purple pixel!");
                    telemetry.addData("Config: ", "Robot will drop Purple Pixel.");
                    dropPurplePixel = true;
                }
            }
        }


    }


}