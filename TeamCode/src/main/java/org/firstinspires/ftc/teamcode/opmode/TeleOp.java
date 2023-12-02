package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.testbot.TestBotDrive;
import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalOpMode.opMode = this;
        TestBotDrive drive = new TestBotDrive(hardwareMap);

        AprilTagCamera[] cameras = new AprilTagCamera[3];
        cameras[0] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Left"), 8, Math.toRadians(70), Math.toRadians(-45));
        cameras[1] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Center"), 7, Math.toRadians(90), Math.toRadians(0));
        cameras[2] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Right"), 8, Math.toRadians(-70), Math.toRadians(45));

        waitForStart();


        while (!isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));

            if (gamepad1.a) {
                //turn on intake or else >:(

            }
            else {
                //turn off intake motor or i come for you
            }
            if (gamepad1.b) {
                //make the release or i EAT YOU
            }
            else {
                //close the pixel hand again
            }
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                //launch the airplane
            }
            else {
                //keep airplane closed or something idk
            }
            if (gamepad1.right_trigger>0.5) {
                //raise bar hang
            }

            if (gamepad1.left_trigger > 0.5) {
                //lower bar hang
            }

        }



    }
}
//hodie christus natus est terra canunt angeli laetantur archangeli alleluia hodie exultant justi dicentes gloria in excelsis deo et in terra pax homnibus bonnae voluntatis alleluia
//et in terra pax homnibus bonnae voluntatis gloria in excelsis deo lauda muste glorifica muste benedicimuste