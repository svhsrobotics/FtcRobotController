package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.panthera.PantheraDrive;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.util.Toggle;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {
    DcMotorEx barHangMotor;
    DcMotorEx flipperMotor;
    DcMotorEx intakeMotor;
    Servo purpleServo;

    double ispeed = 0;

    public static double FLIPPER_POWER = 0.6;
    public static int FLIPPER_POS = -475;



    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        PantheraDrive drive = new PantheraDrive(hardwareMap);
        barHangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "pdw_intake");
        flipperMotor = hardwareMap.get( DcMotorEx.class,"flipper");
        purpleServo = hardwareMap.get(Servo.class, "purple");
        purpleServo.setPosition(0.5);


        barHangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        barHangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flipperMotor.setTargetPosition(flipperMotor.getCurrentPosition());
        flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        Toggle intakeToggle = new Toggle();

        while (!isStopRequested()) {
            drive.update(); // MUST be called every loop cycle so that RoadRunner calculates the pose correctly
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(-gamepad1.left_stick_y * 0.5, -gamepad1.left_stick_x * 0.5).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x * 0.5));

            if (gamepad1.x) {
                drive.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), 0));
            }
            // TODO: Add controls to gamepad 2 as well

            if (gamepad1.b) {
                throwPixel();
            } else if (gamepad1.a) {
                resetFlipper();
            }

//            if (gamepad1.right_bumper && gamepad1.left_bumper && getRuntime() >= 85) { // technically endgame is 90sec, we let them launch a little early just in case
//                // Launch the airplane
//            }

            if (gamepad1.right_trigger > 0.1 && barHangMotor.getCurrentPosition() > -8490) {
                barHangMotor.setPower(-gamepad1.right_trigger * 0.5);
            } else if (gamepad1.left_trigger > 0.1 && barHangMotor.getCurrentPosition() < -50) {
                barHangMotor.setPower(gamepad1.left_trigger * 0.5);
            } else {
                barHangMotor.setPower(0);
            }

            intakeToggle.update(gamepad1.dpad_up);
            if (intakeToggle.state) {
                intakeMotor.setPower(0.6);
            } else if (gamepad1.dpad_down) {
                intakeMotor.setPower(-0.6);
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad1.dpad_left) {
                purpleServo.setPosition(0.5);
            } else if (gamepad1.dpad_right) {
                purpleServo.setPosition(1);// open
            }



            telemetry.addData("ispeed", ispeed);
            telemetry.addData("bar hang", barHangMotor.getCurrentPosition());
            telemetry.addData("flipper ticks", flipperMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    void throwPixel() {
        flipperMotor.setTargetPosition(FLIPPER_POS);
        flipperMotor.setPower(FLIPPER_POWER);
    }

    void resetFlipper() {
        flipperMotor.setTargetPosition(0);
    }
}


