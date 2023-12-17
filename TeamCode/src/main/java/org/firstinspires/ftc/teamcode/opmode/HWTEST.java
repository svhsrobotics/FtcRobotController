package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.testbot.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.testbot.TestBotDrive;

@TeleOp
@Disabled
public class HWTEST extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TestBotDrive drive = new TestBotDrive(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));
            drive.update();

            android.util.Log.i("TESTODO", "LEFT " + ((StandardTrackingWheelLocalizer) drive.getLocalizer()).leftEncoder.getCurrentPosition());
            android.util.Log.i("TESTODO", "RIGHT " + ((StandardTrackingWheelLocalizer) drive.getLocalizer()).rightEncoder.getCurrentPosition());
            android.util.Log.i("TESTODO", "CENTER " + ((StandardTrackingWheelLocalizer) drive.getLocalizer()).frontEncoder.getCurrentPosition());



        }

    }
}
