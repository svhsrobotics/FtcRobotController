package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag Test")
public class ApriltagTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        VisionPortal visionPortal = builder.build();



        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    //double theta_need = detection.ftcPose.yaw + detection.ftcPose.bearing;
                    //double theta_need_rad = Math.toRadians(theta_need);
                    telemetry.addData("Range", detection.ftcPose.range);
                    //telemetry.addData("Theta Need", theta_need);
                    telemetry.addData("Yaw", detection.ftcPose.yaw);
                    telemetry.addData("Bearing", detection.ftcPose.bearing);
                    telemetry.addData("X, Y", detection.ftcPose.x + ", " + detection.ftcPose.y);
                    double thetaNeed = detection.ftcPose.yaw - detection.ftcPose.bearing;
                    telemetry.addData("Theta Need", thetaNeed);
                    double a = detection.ftcPose.range * Math.cos(Math.toRadians(thetaNeed));
                    double b = detection.ftcPose.range * Math.sin(Math.toRadians(thetaNeed));
                    telemetry.addData("a", a);
                    telemetry.addData("b",b);


                    //telemetry.addData("Relative X", detection.ftcPose.range * Math.cos(Math.toRadians(detection.ftcPose.yaw)));
                    //telemetry.addData("Relative Y", detection.ftcPose.range * Math.sin(Math.toRadians(detection.ftcPose.yaw)));

                    //Orientation.getOrientation()
                    //telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));

                    //telemetry.addLine(String.format("Field XYZ %6.1f %6.1f %6.1f  (inch)", detection.metadata.fieldPosition.get(0), detection.metadata.fieldPosition.get(1), detection.metadata.fieldPosition.get(2)));
                }
                telemetry.update();
            }
        }

        visionPortal.close();

    }
}
