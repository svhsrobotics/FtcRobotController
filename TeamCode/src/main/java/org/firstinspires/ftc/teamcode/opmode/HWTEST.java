package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.TrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.TrajectoryDrive;


@TeleOp
// -34502 to -35218 -716
// -18503 to -19233 -730
// -14975 to -14200
public class HWTEST extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        TestBotDrive drive = new TestBotDrive(hardwareMap);
        Robot robot = Robot.thisRobot(hardwareMap);
        TrajectoryDrive drive = robot.getDrive();
        waitForStart();
        int leftI = ((TrackingWheelLocalizer) drive.getLocalizer()).leftEncoder.getCurrentPosition();
        int rightI = ((TrackingWheelLocalizer) drive.getLocalizer()).rightEncoder.getCurrentPosition();
        int frontI = ((TrackingWheelLocalizer) drive.getLocalizer()).frontEncoder.getCurrentPosition();
        while (opModeIsActive()) {
            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));
            drive.update();

            android.util.Log.i("TESTODO", "LEFT " + ((TrackingWheelLocalizer) drive.getLocalizer()).leftEncoder.getCurrentPosition());
            android.util.Log.i("TESTODO", "RIGHT " + ((TrackingWheelLocalizer) drive.getLocalizer()).rightEncoder.getCurrentPosition());
            android.util.Log.i("TESTODO", "CENTER " + ((TrackingWheelLocalizer) drive.getLocalizer()).frontEncoder.getCurrentPosition());
            telemetry.addData("LEFT", ((TrackingWheelLocalizer) drive.getLocalizer()).leftEncoder.getCurrentPosition());
            telemetry.addData("RIGHT", ((TrackingWheelLocalizer) drive.getLocalizer()).rightEncoder.getCurrentPosition());
            telemetry.addData("CENTER", ((TrackingWheelLocalizer) drive.getLocalizer()).frontEncoder.getCurrentPosition());

            telemetry.addData("DELTA LEFT", ((TrackingWheelLocalizer) drive.getLocalizer()).leftEncoder.getCurrentPosition() - leftI);
            telemetry.addData("DELTA RIGHT", ((TrackingWheelLocalizer) drive.getLocalizer()).rightEncoder.getCurrentPosition()- rightI);
            telemetry.addData("DELTA CENTER", ((TrackingWheelLocalizer) drive.getLocalizer()).frontEncoder.getCurrentPosition() - frontI);

            telemetry.addData("DELTA L-R", (((TrackingWheelLocalizer) drive.getLocalizer()).leftEncoder.getCurrentPosition() - leftI)- (((TrackingWheelLocalizer) drive.getLocalizer()).rightEncoder.getCurrentPosition()- rightI));

            telemetry.update();





        }

    }
}
