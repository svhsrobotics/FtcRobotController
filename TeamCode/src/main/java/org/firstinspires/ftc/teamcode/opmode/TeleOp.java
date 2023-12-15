package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.panthera.PantheraDrive;
import org.firstinspires.ftc.teamcode.drive.psi.PsiDrive;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    DcMotorEx barHangMotor;
    DcMotorEx flipperMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        // CHANGE THIS HERE TO CHANGE THE BOT TYPE
        PantheraDrive drive = new PantheraDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);
        barHangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");
        barHangMotor.setTargetPosition(barHangMotor.getCurrentPosition());
        barHangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (robot.botType != Robot.BotType.ROBOTICA) {
            telemetry.addData("Error", "This opmode is only compatible with the PSI bot");
            telemetry.update();
            return;
        }

//        AprilTagCamera[] cameras = new AprilTagCamera[3];
//        cameras[0] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Left"), 8, Math.toRadians(70), Math.toRadians(-45));
//        cameras[1] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Center"), 7, Math.toRadians(90), Math.toRadians(0));
//        cameras[2] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Right"), 8, Math.toRadians(-70), Math.toRadians(45));

        flipperMotor = hardwareMap.get( DcMotorEx.class,"flipper");
        flipperMotor.setTargetPosition(flipperMotor.getCurrentPosition());
        flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();


        while (!isStopRequested()) {
            drive.update(); // MUST be called every loop cycle so that RoadRunner calculates the pose correctly
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));

            if (gamepad1.x) {
                drive.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), 0));
            }
            // TODO: Add controls to gamepad 2 as well
            if (gamepad1.a) {
                // Intake running while A is held

            } else {
                // Turn off intake when A released
            }

            if (gamepad1.b) {
                // Move pixel hand servo to release 1 pixel
                // TODO: Do we need to debounce and count presses? Svit mentioned a "pixel cassette" that drops multiple pixels
                throwPixel();
            } else {
                // Close pixel hand servo
                //resetFlipper();
            }
            if (gamepad1.a) {
                resetFlipper();
            }
            if (gamepad1.right_bumper && gamepad1.left_bumper && getRuntime() >= 85) { // technically endgame is 90sec, we let them launch a little early just in case
                // Launch the airplane
            }

            // Set bar hang motor power to gamepad1.right_trigger - gamepad1.left_trigger for analog control

//            if (gamepad1.right_trigger >.1 && barHangMotor.getTargetPosition() < 538 ) {
//
//            }
            if (gamepad1.y) {
                barHang();
            }

        }





    }
    void barHang() {

        barHangMotor.setTargetPosition(538);
        barHangMotor.setPower(0.2);


    }
    int startFlipperTicks=0;

    void throwPixel() {
        int flipperTicks = flipperMotor.getCurrentPosition();
        if (startFlipperTicks==0) {
           startFlipperTicks=flipperTicks;
        }
        flipperMotor.setTargetPosition(flipperTicks-475);
        flipperMotor.setPower(0.1);
        //flipperMotor.
        telemetry.addData("flipper ticks", flipperTicks);
        telemetry.update();

    }
    void resetFlipper() {
        flipperMotor.setTargetPosition(startFlipperTicks);
    }
    //alelluia deus omnipotens confitor deo omnipotenti

    //in nomine patris, et fili, et spiritu sancti



}


