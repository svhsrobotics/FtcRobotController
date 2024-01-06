package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.drive.panthera.PantheraDriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.panthera.PantheraDriveConstants.MAX_ANG_VEL;

import static org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer.Quadrant.*;
import static org.firstinspires.ftc.teamcode.vision.TensorFlowDetection.PropPosition.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.panthera.PantheraDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.TensorFlowDetection;

public class AutoPaths {
    static Vector2d BOT_DROPPER_OFFSET = new Vector2d(4,4.5); // PANTHERA

    /* =========== BLUE =========== */
    static Vector2d BLUE_PARK = new Vector2d(48, 48);

    /* BOARD */
    static Vector2d BLUE_BOARD_CENTER_LINE = new Vector2d(12, 24.5);
    static Vector2d BLUE_BOARD_LEFT_LINE = new Vector2d(23, 30);
    static Vector2d BLUE_BOARD_RIGHT_LINE = new Vector2d(1, 30);

    /* AUDIENCE */

    /* =========== RED =========== */
    static Vector2d RED_PARK = new Vector2d(48, -48);

    /* BOARD */
    static Vector2d RED_BOARD_CENTER_LINE = new Vector2d(12, -24.5);
    // LEFT MISSING

    static Vector2d RED_BOARD_RIGHT_LINE = new Vector2d(23, -30);


    /* AUDIENCE */

    private static Vector2d lineCoords(AprilTagLocalizer.Quadrant quadrant, TensorFlowDetection.PropPosition prop) {
        switch (quadrant) {
            case BLUE_BOARD:
                switch (prop) {
                    case CENTER:
                        return BLUE_BOARD_CENTER_LINE;
                    case LEFT:
                        return BLUE_BOARD_LEFT_LINE;
                    case RIGHT:
                        return BLUE_BOARD_RIGHT_LINE;
                }
            case BLUE_AUDIENCE:
                switch (prop) {
                    case CENTER:
                        return null;
                    case LEFT:
                        return null;
                    case RIGHT:
                        return null;
                }
            case RED_BOARD:
                switch (prop) {
                    case CENTER:
                        return RED_BOARD_CENTER_LINE;
                    case LEFT:
                        return null;
                    case RIGHT:
                        return RED_BOARD_RIGHT_LINE;
                }
            case RED_AUDIENCE:
                switch (prop) {
                    case CENTER:
                        return null;
                    case LEFT:
                        return null;
                    case RIGHT:
                        return null;
                }
        }
        return null;
    }
    public static TrajectorySequence buildTrajectory(
            Pose2d startPose,
            AprilTagLocalizer.Quadrant quadrant,
            TensorFlowDetection.PropPosition prop) {
        TrajectorySequenceBuilder trajBuilder = new TrajectorySequenceBuilder(
                startPose,
                // TODO: Make the constraints configurable
                PantheraDrive.VEL_CONSTRAINT, PantheraDrive.ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
        // If we're on the RED side, flip the offset
        if (quadrant == RED_BOARD || quadrant == RED_AUDIENCE) {
            BOT_DROPPER_OFFSET = BOT_DROPPER_OFFSET.times(-1);
        }
        // If it's a CENTER prop, move forward to the coords
        if (prop == CENTER) {
            trajBuilder.lineTo(lineCoords(quadrant, prop));
        }
//        builder.lineTo(new Vector2d(BLUE_BOARD_LEFT_LINE.getX() - BOT_DROPPER_OFFSET.getX(), BLUE_BOARD_LEFT_LINE.getY() + BOT_DROPPER_OFFSET.getY()))
//                // Drop the pixel
//                .addTemporalMarker(()->{
//                    //PIXEL DROP
//                    Log.i("DROP", "dropping purple");
//                    purpleServo.setPosition(1);
//                })
        trajBuilder.waitSeconds(1.0)
                // Back up to the BLUE PARK Y coord
                .lineTo(new Vector2d(BLUE_BOARD_LEFT_LINE.getX(), BLUE_PARK.getY()))
                // Face forwards
                .turn(Math.toRadians(90))
                // Park
                .lineTo(BLUE_PARK)
                .build();
        return trajBuilder.build();
    }
}
