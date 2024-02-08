package org.firstinspires.ftc.teamcode.drive;

//import static org.firstinspires.ftc.teamcode.drive.testbot.DriveConstants.maxAccel;
//import static org.firstinspires.ftc.teamcode.drive.testbot.DriveConstants.maxAngAccel;
//import static org.firstinspires.ftc.teamcode.drive.testbot.DriveConstants.maxAngVel;
//import static org.firstinspires.ftc.teamcode.drive.testbot.DriveConstants.maxVel;
//import static org.firstinspires.ftc.teamcode.drive.testbot.DriveConstants.MOTOR_VELO_PID;
//import static org.firstinspires.ftc.teamcode.drive.testbot.DriveConstants.RunUsingEncoder;
//import static org.firstinspires.ftc.teamcode.drive.testbot.DriveConstants.encoderTicksToInches;
//import static org.firstinspires.ftc.teamcode.drive.testbot.DriveConstants.kA;
//import static org.firstinspires.ftc.teamcode.drive.testbot.DriveConstants.kStatic;
//import static org.firstinspires.ftc.teamcode.drive.testbot.DriveConstants.kV;
//import static org.firstinspires.ftc.teamcode.drive.testbot.DriveConstants.maxAngAccel;
//import static org.firstinspires.ftc.teamcode.drive.testbot.DriveConstants.maxAngVel;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class TrajectoryDrive extends MecanumDrive {
    //public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(6, 0, 1.05);
    //public static PIDCoefficients HEADING_PID = new PIDCoefficients(4, 0, 0);

    //public static double lateralMultiplier = 1.05564102564;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;



    private TrajectoryFollower follower;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    private final TrajectoryAccelerationConstraint accelerationConstraint;
    private final TrajectoryVelocityConstraint velocityConstraint;

    private final double encoderTicksMultiplier;
    private final double maxAngAccel;
    private final double maxAngVel;

    public double encoderTicksToInches(double ticks) {
        return encoderTicksMultiplier * ticks;
    }

    public TrajectoryDrive(HardwareMap hardwareMap,
                           PIDCoefficients translationalPid,
                           PIDCoefficients headingPid,
                           double lateralMultiplier,
                           String leftFrontDriveName,
                           String leftRearDriveName,
                           String rightRearDriveName,
                           String rightFrontDriveName,
                           PIDFCoefficients motorVeloPID,
                           double maxAccel,
                           double maxAngAccel,
                           double maxAngVel,
                           double maxVel,
                           boolean runUsingEncoder,
                           double trackWidth,
                           double encoderTicksMultiplier,
                           double kA,
                           double kV,
                           double kStatic,
                           String leftEncoderName,
                           String rightEncoderName,
                           String perpEncoderName,
                           double x_mult,
                           double y_mult,
                           double forward_offset,
                           double lateral_distance,
                           double gear_ratio,
                           double wheel_radius,
                           double ticks_per_rev



    ) {

        super(kV, kA, kStatic, trackWidth, trackWidth, lateralMultiplier);
        velocityConstraint = getVelocityConstraint(maxVel, maxAngVel, trackWidth);
        accelerationConstraint = getAccelerationConstraint(maxAccel);
        this.encoderTicksMultiplier = encoderTicksMultiplier;
        this.maxAngAccel = maxAngAccel;
        this.maxAngVel = maxAngVel;

        follower = new HolonomicPIDVAFollower(translationalPid, translationalPid, headingPid,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        //imu = hardwareMap.get(IMU.class, "imu");
        //IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        //        PantheraDriveConstants.LOGO_FACING_DIR, PantheraDriveConstants.USB_FACING_DIR));
        //imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontDriveName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearDriveName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearDriveName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontDriveName);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (runUsingEncoder) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (runUsingEncoder && motorVeloPID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, motorVeloPID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
        setLocalizer(new TrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels,
                leftEncoderName,
                rightEncoderName, perpEncoderName,
        x_mult,
        y_mult,
        forward_offset,
        lateral_distance,
        gear_ratio,
        wheel_radius,
        ticks_per_rev
        ));



        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, headingPid, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velocityConstraint, accelerationConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velocityConstraint, accelerationConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velocityConstraint, accelerationConstraint);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                velocityConstraint, accelerationConstraint,
                maxAngVel, maxAngAccel
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
 //       return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
    //    return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        return 0.0;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public enum Quadrant {
        RED_AUDIENCE,
        RED_BOARD,
        BLUE_AUDIENCE,
        BLUE_BOARD
    }

    public static TrajectoryDrive.Quadrant whichQuadrant(Pose2d pose) {
        if (pose.getX() < 0) {
            if (pose.getY() < 0) {
                return TrajectoryDrive.Quadrant.RED_AUDIENCE;
            } else {
                return TrajectoryDrive.Quadrant.BLUE_AUDIENCE;
            }
        } else {
            if (pose.getY() < 0) {
                return TrajectoryDrive.Quadrant.RED_BOARD;
            } else {
                return TrajectoryDrive.Quadrant.BLUE_BOARD;
            }
        }
    }

    public Quadrant currentQuadrant() {
        return whichQuadrant(getPoseEstimate());
    }
}
