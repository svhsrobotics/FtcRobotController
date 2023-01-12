package org.firstinspires.ftc.teamcode.Shared;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.util.HashMap;

public class Navigator {
    // Used to when calling Android's Log class
    String TAG = "Drive";

    /// These are fetched from the Robot
    // TODO: Use the Robot class directly (maybe refactor to be more concise)
    public final org.firstinspires.ftc.teamcode.robot.hardware.Drive leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;

    /// This is only used in order to call isStopRequested() during the navigationMonitorTicks loop, and for the telemetry
    LinearOpMode opMode;
    // TODO: This should replaced with a Logger
    Telemetry telemetry;
    Logger logger = new Logger();

    // TODO: Replace this with a local timer variable
    private final ElapsedTime runtime = new ElapsedTime();

    // These values should be replaced in the constructor
    final double COUNTS_PER_MOTOR_REV;  /// Ticks output by the motor encoder for each revolution of the motor shaft
    final double DRIVE_GEAR_REDUCTION;  /// Modifier, less than 1 if geared UP
    final double WHEEL_DIAMETER_INCHES; /// Self-explanatory
    final double COUNTS_PER_INCH;       /// Translates encoder ticks to inches

    /// These multipliers are used to scale the power of the motors based on the overall power
    HashMap <org.firstinspires.ftc.teamcode.robot.hardware.Drive, Double> motorPowerFactors;

    /// The robot class, contains all the hardware
    Robot robot;

    // This is used to stop the robot from a different thread, set in ceaseMotion and used in the navigationMonitorTicks loop
    boolean mIsStopped = false;

    final double SPEEDSCALE = 50;//Speed in inches/second at speed=1

    public Navigator(Robot robot, LinearOpMode opMode){
        this.robot = robot;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.logger = new Logger(opMode.telemetry);

        this.leftFrontDrive = robot.drives.get(Robot.DrivePos.FRONT_LEFT);
        this.rightFrontDrive = robot.drives.get(Robot.DrivePos.FRONT_RIGHT);
        this.leftBackDrive = robot.drives.get(Robot.DrivePos.BACK_LEFT);
        this.rightBackDrive = robot.drives.get(Robot.DrivePos.BACK_RIGHT);

        // Replace default values with actual values
        COUNTS_PER_MOTOR_REV = robot.REV_COUNTS;
        DRIVE_GEAR_REDUCTION = robot.GEAR_REDUCTION;
        WHEEL_DIAMETER_INCHES = robot.WHEEL_DIAMETER;
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        motorPowerFactors = new HashMap<>();
        setTargetAngle(0);
    }

    /**navigationByPhi
     *
     * @param targetSpeed Desired Speed
     * @param targetTheta Desired Orientation of Robot
     *
     * Utilizes orientation away from the desired location(phi) in order to alter power power facotrs on the motors.
     */
    public void navigationByPhi(double targetSpeed, double targetTheta) {
        double rightFrontPowerFactor, leftFrontPowerFactor, rightBackPowerFactor, leftBackPowerFactor;
        double pi = Math.PI;

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path",  "Starting at %7d : %7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.update();

        // reset the timeout time and start motion.
        runtime.reset();

        if(targetTheta > 0 && targetTheta < pi/2){
            rightFrontPowerFactor = -Math.cos(2 * targetTheta);
        }else if(targetTheta >= -pi && targetTheta < -pi/2){
            rightFrontPowerFactor = Math.cos(2 * targetTheta);
        }else if(targetTheta >= pi/2 && targetTheta <= pi){
            rightFrontPowerFactor = 1;
        }else{
            rightFrontPowerFactor = -1;
        }

        if(targetTheta > 0 && targetTheta < pi/2) {
            leftBackPowerFactor = -Math.cos(2 * targetTheta);
        }else if(targetTheta >= -pi && targetTheta < -pi/2){
            leftBackPowerFactor = Math.cos(2 * targetTheta);
        }else if(targetTheta >= pi/2 && targetTheta <= pi){
            leftBackPowerFactor = 1;
        }else{
            leftBackPowerFactor = -1;
        }

        if(targetTheta > -pi/2 && targetTheta < 0) {
            rightBackPowerFactor = Math.cos(2 * targetTheta);
        }else if(targetTheta > pi/2 && targetTheta < pi){
            rightBackPowerFactor = -Math.cos(2 * targetTheta);
        }else if(targetTheta >= 0 && targetTheta <= pi/2){
            rightBackPowerFactor = 1;
        }else{
            rightBackPowerFactor = -1;
        }

        if(targetTheta > -pi/2 && targetTheta < 0) {
            leftFrontPowerFactor = Math.cos(2 * targetTheta);
        }else if(targetTheta > pi/2 && targetTheta < pi){
            leftFrontPowerFactor = -Math.cos(2 * targetTheta);
        }else if(targetTheta >= 0 && targetTheta <= pi/2){
            leftFrontPowerFactor = 1;
        }else{
            leftFrontPowerFactor = -1;
        }

        motorPowerFactors.put(leftFrontDrive, leftFrontPowerFactor);
        motorPowerFactors.put(leftBackDrive, leftBackPowerFactor);
        motorPowerFactors.put(rightFrontDrive, rightFrontPowerFactor);
        motorPowerFactors.put(rightBackDrive, rightBackPowerFactor);

        SpeedsPhi speedsPhi = getPhiSpeeds(targetSpeed, motorPowerFactors);
        setMotorPowersPhi(speedsPhi);
    }

    public void navigationMonitorTicksAndCeaseMotion(double speed, double xInches, double yInches, double timeout) {
        navigationMonitorTicks(speed, xInches, yInches, timeout);
        ceaseMotion();
    }


    /**
     * This is the original navigation function, without anything fancy.
     * @param speed inches per second
     * @param xInches inches left/right
     * @param yInches inches forward/backwards
     * @param timeout seconds
     */
    public void navigationMonitorTicks(double speed, double xInches, double yInches, double timeout) {
        navigationMonitorTicksPhi(speed, xInches, yInches, 0, timeout);
    }

    /**
     * This is a slightly different version that adds phi rotation to the mix.
     * Please don't try to move x/y *and* rotate just yet...
     * @param speed inches per second
     * @param xInches inches left/right
     * @param yInches inches forward/backwards
     * @param phi degrees heading (relative)
     * @param timout seconds
     */
    public void navigationMonitorTicksPhi(double speed, double xInches, double yInches, double phi, double timout) {
        internalNavigationLoop(speed, xInches, yInches, phi, timout, false, true);
    }

    public void navigationMonitorTurn(double phi) {
        // We only want to turn, but due to a bug in the navigation loop, it will not work if both x/y are 0.
        // So we'll set x to 1, and the speed to 0 so that it doesn't actually move.
        // monitorTotalTravel is set to false so that the loop is not blocked until the correct number of ticks are reached (which will never happen)
        internalNavigationLoop(0, 1, 0, phi, 10, false, false);
    }

    public void navigationMonitorExternal(double inchesPerSecond, double xInches, double yInches, double phi, double timeoutSec, boolean isMonitorAcceleration) {
        internalNavigationLoop(inchesPerSecond, xInches, yInches, phi, timeoutSec, isMonitorAcceleration, true);
    }

    /**
     * INTERNAL USE ONLY. Takes care of the navigation loop, lots of parameters that should be engaged with other functions.
     */
    private void internalNavigationLoop(double inchesPerSecond, double xInches, double yInches, double phi, double timeoutSec, boolean isMonitorAcceleration, boolean monitorTotalTravel) {
        //Borrowed Holonomic robot navigation ideas from https://www.bridgefusion.com/blog/2019/4/10/robot-localization-dead-reckoning-in-f  irst-tech-challenge-ftc
        //    Robot Localization -- Dead Reckoning in First Tech Challenge (FTC)
        Log.i("start", "#$#$#$#$#$#$#$#$#$");
        double theta = Math.atan2(yInches, xInches);
        double magnitude = Math.hypot(xInches, yInches);
        int tickCountPriorLeftFront = leftFrontDrive.getCurrentPosition(), tickCountPriorLeftBack = leftBackDrive.getCurrentPosition();
        int tickCountPriorRightFront = rightFrontDrive.getCurrentPosition(), tickCountPriorRightBack = rightBackDrive.getCurrentPosition();
        int ticksTraveledLeftFront = 0, ticksTraveledLeftBack = 0, ticksTraveledRightFront = 0, ticksTraveledRightBack = 0;
        double inchesTraveledX = 0, inchesTraveledY = 0, inchesTraveledTotal = 0, rotationInchesTotal = 0;
        long cycleMillisNow, cycleMillisPrior = System.currentTimeMillis(), cycleMillisDelta, startMillis = System.currentTimeMillis();

        // Get the time it is right now, so we can start the timer
        //long start = System.currentTimeMillis();

        //vroom_vroom(speed, theta, speed, theta);
        mIsStopped = false;
        mTargetAngleErrorSum = 0;
        getImuAngle();
        double speed = inchesPerSecond/SPEEDSCALE;
        navigationByPhi(speed, theta);
        adjustThetaInit();
        //setTargetAngle(mImuCalibrationAngle);

        // We fudge the angle a little, because something is wrong...
        // TODO: Investigate this fudge
        double angle_fudged = phi - (phi * (1.5 / 90.0));
        setTargetAngle(angle_fudged);

        int settleCounter = 0;

        // Stay in the main loop while:
        while (opMode.opModeIsActive() // The opmode is still active
                && System.currentTimeMillis() < startMillis + (1000 * timeoutSec) // We haven't timed out
                && (monitorTotalTravel && (inchesTraveledTotal < magnitude)) // We are monitoring total travel, and we haven't reached the target yet
                || (settleCounter < 3) // We are waiting for the robot to settle
                && !mIsStopped // We haven't been told to stop (by another thread)
                && !shouldStopIfApplicable(isMonitorAcceleration, startMillis) // We haven't been told to stop (by the acceleration monitor)
        ) {
            if (getEulerAngleDegrees(mTargetAngle - getAdjustedAngle()) < 1) {
                // If we're really close to the target, increase the settle counter
                settleCounter++;
            } else {
                // If we're not close, reset the settle counter
                settleCounter = 0;
            }

            double TotalMotorCurrent = leftFrontDrive.getCurrent();
            TotalMotorCurrent += leftBackDrive.getCurrent();
            TotalMotorCurrent += rightFrontDrive.getCurrent();
            TotalMotorCurrent += rightBackDrive.getCurrent();
            Log.i(TAG,"navigationMonitorTicks: Total Motor Current= "+ TotalMotorCurrent);


            int tickCountNowLeftFront = leftFrontDrive.getCurrentPosition();
            int tickCountNowLeftBack = leftBackDrive.getCurrentPosition();
            int tickCountNowRightFront = rightFrontDrive.getCurrentPosition();
            int tickCountNowRightBack = rightBackDrive.getCurrentPosition();
            int deltaTicksLeftFront = tickCountNowLeftFront - tickCountPriorLeftFront;
            int deltaTicksLeftBack = tickCountNowLeftBack - tickCountPriorLeftBack;
            int deltaTicksRightFront = tickCountNowRightFront - tickCountPriorRightFront;
            int deltaTicksRightBack = tickCountNowRightBack - tickCountPriorRightBack;
            ticksTraveledLeftFront += deltaTicksLeftFront;
            ticksTraveledLeftBack += deltaTicksLeftBack;
            ticksTraveledRightFront += deltaTicksRightFront;
            ticksTraveledRightBack += deltaTicksRightBack;
            double leftFrontInchesDelta = deltaTicksLeftFront / COUNTS_PER_INCH;
            double rightFrontInchesDelta = -deltaTicksRightFront / COUNTS_PER_INCH;  //Minus sign converts to holonomic drive perspective
            double rightBackInchesDelta = -deltaTicksRightBack / COUNTS_PER_INCH;  //Minus sign converts to holonomic drive perspective
            double leftBackInchesDelta = deltaTicksLeftBack / COUNTS_PER_INCH;
            double rotationAvgInchesDelta = (leftFrontInchesDelta + rightFrontInchesDelta + rightBackInchesDelta + leftBackInchesDelta)/4;
            rotationInchesTotal += rotationAvgInchesDelta;
            double leftFrontRobotInchesDelta = leftFrontInchesDelta - rotationAvgInchesDelta;
            double rightFrontRobotInchesDelta = rightFrontInchesDelta - rotationAvgInchesDelta;
            double rightBackRobotInchesDelta = rightBackInchesDelta - rotationAvgInchesDelta;
            double leftBackRobotInchesDelta = leftBackInchesDelta - rotationAvgInchesDelta;
            double deltaInchesRobotX = (leftFrontRobotInchesDelta + rightFrontRobotInchesDelta - rightBackRobotInchesDelta - leftBackRobotInchesDelta) / (2 * Math.sqrt(2));
            double deltaInchesRobotY = (leftFrontRobotInchesDelta - rightFrontRobotInchesDelta - rightBackRobotInchesDelta + leftBackRobotInchesDelta) / (2 * Math.sqrt(2));
            double deltaInchesRobot = Math.hypot(deltaInchesRobotX, deltaInchesRobotY);
            double FUDGE_FACTOR = ((36/51.0)*(38/32.0));
            inchesTraveledX += deltaInchesRobotX * FUDGE_FACTOR;
            inchesTraveledY += deltaInchesRobotY * FUDGE_FACTOR;
            inchesTraveledTotal += Math.hypot(deltaInchesRobotX * FUDGE_FACTOR, deltaInchesRobotY * FUDGE_FACTOR);

            cycleMillisNow = System.currentTimeMillis();
            cycleMillisDelta = cycleMillisNow - cycleMillisPrior;
            cycleMillisPrior = cycleMillisNow;


//Working on Changing Speed to be consistent

            if(runtime.seconds()>.05 && inchesTraveledTotal <= magnitude) {
                //  timePassed = opMode.getRuntime() - initialTime;//Length of time of the loop
                double actualSpeedX = 1000 * (deltaInchesRobotX / cycleMillisDelta);//gets the actual speed in inches/second in x direction
                double actualSpeedY = 1000 * (deltaInchesRobotY / cycleMillisDelta);//gets the actual speed in inches/second in y direction
                //SPEEDSCALE = 24.0;//Speed in inches/second at speed=1
                //speedScaled = speed * SPEEDSCALE;//Speed in terms of inches/second instead of 0 to 1


                double actualSpeed = Math.sqrt((actualSpeedX * actualSpeedX) + (actualSpeedY * actualSpeedY));//Take hypotenuse of speed in x and y
                Log.i("Speed", ".........................................................................................");
                Log.i("Speed", String.format("InchesDeltaX, %.3f, InchesDeltaY, %.3f", deltaInchesRobotX, deltaInchesRobotY));

                Log.i("Speed", String.format("Actual Speed in/s:, %.2f", actualSpeed));//log actual speed

                Log.i("Speed", String.format("SpeedScale:, %.1f", SPEEDSCALE));
                Log.i("Speed", String.format("Target Speed in/s:, %.1f", inchesPerSecond));
                //  Log.i("Speed", String.format("SpeedScaled in/s:, %.2f", speedScaled));//log desired speed scaled should be input speed*12

                double speedError = (inchesPerSecond - actualSpeed);//error in speed in/s
                Log.i("Speed", String.format("SpeedError in/s:, %.2f", speedError));
                double speedGain = 0.005*.1;
                if (speedError >= 1.25 * inchesPerSecond) {
                    speedError = 1.25 * inchesPerSecond;
                } else if (speedError <= -1.25 * inchesPerSecond) {
                    speedError = -1.25 * inchesPerSecond;
                }
                speed = speed + speedGain * speedError;
                //Log.i("Speed", String.format("speedFactor in/s:, %.2f", speedFactor));
                // speedScaled *= (1 + speedFactor);//corrects adjusted speed with percent error

                //Log.i("Speed", String.format("New Speed in/s:, %.2f", speed));//log actual speed
                // speed = speedScaled / SPEEDSCALE;//converts adjusted speed to 0 to 1 scale
                Log.i("Speed", String.format("New Speed from 0 to 1:, %.2f", speed));//log actual speed
                Log.i("Speed", ".........................................................................................");
                //Provide feedback to keep robot moving in right direction based on encoder ticks


                adjustTheta(xInches, yInches, speed, inchesTraveledX, inchesTraveledY);
            }

            //Must call adjustThetaInit() before a loop with adjustTheta()



            telemetry.addData("Ticks Traveled (lf, lb)", "%7d, %7d", ticksTraveledLeftFront, ticksTraveledLeftBack);
            telemetry.addData("Ticks Traveled (rf, rb)", "%7d, %7d", ticksTraveledRightFront, ticksTraveledRightBack);
            telemetry.addData("In Traveled (X, Y)", "X: %.1f, Y: %.1f", inchesTraveledX, inchesTraveledY);
            telemetry.addData("In Traveled (Tot, Rot)", "%.1f, %.1f", inchesTraveledTotal,rotationInchesTotal);
            telemetry.addData("Cycle Millis:", "%d", cycleMillisDelta);
            telemetry.update();
            Log.i("Drive", String.format("Ticks Traveled (lf, lb): %7d, %7d", ticksTraveledLeftFront, ticksTraveledLeftBack));
            Log.i("Drive", String.format("Ticks Traveled (rf, rb): %7d, %7d", ticksTraveledRightFront, ticksTraveledRightBack));
            Log.i("Drive", String.format("Ticks Delta (lf, lb): %7d, %7d", deltaTicksLeftFront, deltaTicksLeftBack));
            Log.i("Drive", String.format("Ticks Delta (rf, rb): %7d, %7d", deltaTicksRightFront, deltaTicksRightBack));
            Log.i("Drive", String.format("In Traveled (X, Y): X: %.2f, Y: %.2f", inchesTraveledX, inchesTraveledY));
            Log.i("Drive", String.format("In Traveled (Tot, Rot): %.2f, %.2f", inchesTraveledTotal,rotationInchesTotal));
            Log.i("Drive", String.format("Incremental Speed (in/sec): %.2f", deltaInchesRobot/cycleMillisDelta * 1000));
            Log.i("Drive", String.format("Cycle Millis: %d, Total Seconds: %d", cycleMillisDelta, (System.currentTimeMillis() - startMillis)/1000));

            tickCountPriorLeftFront = tickCountNowLeftFront;
            tickCountPriorLeftBack = tickCountNowLeftBack;
            tickCountPriorRightFront = tickCountNowRightFront;
            tickCountPriorRightBack = tickCountNowRightBack;
        }
    }

    /**ceaseMotion
     *Stop all motion;
     */
    public void ceaseMotion(){
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mIsStopped = true;
        opMode.sleep(30);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /**setMotorPowersPhi
     *Set power factors of all drive motors using speedsPhi
     */
    public void setMotorPowersPhi(SpeedsPhi speedsPhi){
        leftFrontDrive.setPower(speedsPhi.leftFrontSpeed);
        rightFrontDrive.setPower(speedsPhi.rightFrontSpeed);
        leftBackDrive.setPower(speedsPhi.leftBackSpeed);
        rightBackDrive.setPower(speedsPhi.rightBackSpeed);
    }

    private double thetaErrorSum;
    public void adjustThetaInit() { thetaErrorSum = 0; }
    public void adjustTheta(double targetX, double targetY, double targetSpeed, double nowX, double nowY){
        // TODO: Why is this testing the target angle
        if(nowX == 0 && nowY == 0 && mTargetAngle == 0){
            Log.i("Drive", "adjustTheta: nowX and nowY are both still zero so not computing an adjustment factor yet");
            return;
        }
        //double GAIN = 0.6, THETA_ERROR_SUM_MAX = Math.PI/4/GAIN; //Max error sum is +/- 45 degrees
        double targetTheta = Math.atan2(targetY, targetX);
        double nowTheta = Math.atan2(nowY, nowX);
//        if((targetTheta - nowTheta > 0 && thetaErrorSum < THETA_ERROR_SUM_MAX) || (targetTheta - nowTheta < 0 && thetaErrorSum > -THETA_ERROR_SUM_MAX))
//            thetaErrorSum += Math.min(Math.max(targetTheta - nowTheta, -THETA_ERROR_SUM_MAX), THETA_ERROR_SUM_MAX);  //Max allowed increment is max value
//        double adjustedTargetTheta = targetTheta + GAIN * thetaErrorSum;

        //The cosine function acts as the gain so that as the error angle approaches 180 degrees, the error factor goes to zero,
        // since the motors will already be pulling in the opposite direction.
        double angleDifference = calculateAngleDifference(targetTheta, nowTheta);
        double adjustedTargetTheta = getEulerAngle(targetTheta + Math.cos(angleDifference/2) * angleDifference);
        //Speeds speeds = getSpeeds(targetSpeed, nowTheta);
//        vroom_vroom(targetSpeed, adjustedTargetTheta, targetSpeed, adjustedTargetTheta);
        //vroom_vroom(speeds.rightSpeed, adjustedTargetTheta, speeds.leftSpeed, adjustedTargetTheta);
        navigationByPhi(targetSpeed, adjustedTargetTheta);
//        vroom_vroom(targetSpeed, targetTheta, targetSpeed, targetTheta);
        Log.i("Drive", String.format("IMU angle: %.2f, adj angle: %.2f", mCurrentImuAngle, mAdjustedAngle));
        Log.i("Drive", String.format("adjustTheta: (degrees) target: %.4f, now: %.4f, adjusted: %.4f, error: %.4f",
                targetTheta/Math.PI*180, nowTheta/Math.PI*180, adjustedTargetTheta/Math.PI*180, thetaErrorSum/Math.PI*180));
    }

    double mCurrentImuAngle;
    double mPriorImuAngle;
    double mTargetAngle;
    double mAdjustedAngle;
    double mPriorAdjustedAngle;
    double mTargetAngleErrorSum;

    public void setTargetAngle(double targetAngle){
        mPriorImuAngle = mTargetAngle = targetAngle;
    }

    /**
     *We experimentally determined the Z axis is the axis we want to use for heading angle.
     *We have to process the angle because the imu works in euler angles so the Z axis is
     *returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
     *180 degrees. We detect this transition and track the total cumulative angle of rotation.
     * @return 0
     */
    public double getImuAngle(){
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        telemetry.addData("IMU Angles (X/Y/Z)", "%.1f / %.1f / %.1f", angles.secondAngle, angles.thirdAngle, angles.firstAngle);
        telemetry.update();
        Log.i(TAG, String.format("getImuAngle: IMU Angles (X/Y/Z): %.1f / %.1f / %.1f", angles.secondAngle, angles.thirdAngle, angles.firstAngle));
        return mCurrentImuAngle = angles.firstAngle;

    }

    /**
     *
     * @return New adjusted angle used to create adjusted power levels
     */
    public double getAdjustedAngle(){
        return mAdjustedAngle = getEulerAngleDegrees(getImuAngle());
    }

    /**
     * Convert an angle such that -pi <= angle <= pi
     */
    private double getEulerAngle(double angle){
        double modAngle = angle%(2*Math.PI);
        if(modAngle < -Math.PI) return modAngle + 2 * Math.PI;
        else if (modAngle > Math.PI) return modAngle - 2 * Math.PI;
        else return modAngle;
        //if(angle < -Math.PI) return angle%(2*Math.PI) + 2 * Math.PI;
        //else if (angle > Math.PI) return angle%(2*Math.PI) - 2 * Math.PI;
        //else return angle;
    }

    /**
     * Convert an angle such that -180 <= angle <= 180
     */
    private double getEulerAngleDegrees(double angle){
        double modAngle = angle%360;
        if(modAngle < -180) return modAngle + 360;
        else if (modAngle > 180) return modAngle - 360;
        else return modAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double getPowerCorrection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double angleError, powerCorrection, angle, pGain, iGain;

        angle = getAdjustedAngle();  //IMU angle converted to Euler angle (IMU may already deliver Euler angles)
        angleError = getEulerAngleDegrees(mTargetAngle - angle);        // reverse sign of angle for correction.
        mTargetAngleErrorSum += angleError;
        int MAX_ERROR_ANGLE_SUM = 3;
        if(mTargetAngleErrorSum > MAX_ERROR_ANGLE_SUM)
            mTargetAngleErrorSum = MAX_ERROR_ANGLE_SUM;
        else if(mTargetAngleErrorSum < -MAX_ERROR_ANGLE_SUM)
            mTargetAngleErrorSum = -MAX_ERROR_ANGLE_SUM;
        Log.i(TAG, String.format("getPowerCorrection: targetAngle: %.2f, imuAngle: %.2f, diffAngle: %.2f, diffAngleSum: %.2f", mTargetAngle, angle, angleError, mTargetAngleErrorSum));

        //gain = Math.max(-0.05*Math.abs(angleError) + 0.1, .05);  //Varies from .2 around zero to .05 for errors above 10 degrees
        //pGain = Math.max(-0.05*Math.abs(angleError) + 0.1, .05)/3;  //Varies from .2 around zero to .05 for errors above 10 degrees
        pGain = 0.06 / 8.0;
        iGain = 0.04 / 8.0;

        // If the angleError is really big, go open-loop for a bit to speed up the turn
        //if (Math.abs(angleError) > 20) {
        //    pGain += 1000;
        //}

        powerCorrection = angleError * pGain + mTargetAngleErrorSum * iGain;

        android.util.Log.d("angleError", String.valueOf(angleError));
        android.util.Log.d("mTargetAngleErrorSum", String.valueOf(mTargetAngleErrorSum));
        android.util.Log.d("mTargetAngle", String.valueOf(mTargetAngle));

        /*telemetry.addData("angleError", angleError);
        telemetry.addData("mTargetAngleErrorSum", mTargetAngleErrorSum);
        telemetry.addData("mTargetAngle", mTargetAngle);*/

        //logger.debug("power correction: " + powerCorrection);

        return powerCorrection;
    }

    private static class SpeedsPhi{
        public double rightFrontSpeed;
        public double leftFrontSpeed;
        public double rightBackSpeed;
        public double leftBackSpeed;
        public SpeedsPhi(double leftFront, double leftBack, double rightFront, double rightBack){
            leftFrontSpeed = leftFront;
            leftBackSpeed = leftBack;
            rightFrontSpeed = rightFront;
            rightBackSpeed = rightBack;
        }
    }

    /**
     * Get the adjusted speeds for each side of the robot to allow it to turn enough to stay on a straight line. Only call once per cycle.
     */
    private SpeedsPhi getPhiSpeeds(double targetSpeed, HashMap<org.firstinspires.ftc.teamcode.robot.hardware.Drive, Double> motorPowerFactors){
        double powerCorrection = getPowerCorrection();
        double adjustedLeftFrontSpeed, adjustedLeftBackSpeed, adjustedRightFrontSpeed, adjustedRightBackSpeed;

        if(motorPowerFactors.get(leftFrontDrive)*targetSpeed - powerCorrection > 1){
            adjustedLeftFrontSpeed = 1;
        }else if(motorPowerFactors.get(leftFrontDrive)*targetSpeed - powerCorrection < -1) {
            adjustedLeftFrontSpeed = -1;
        }else{
            adjustedLeftFrontSpeed = motorPowerFactors.get(leftFrontDrive) * targetSpeed - powerCorrection;
        }

        if(motorPowerFactors.get(leftBackDrive)*targetSpeed - powerCorrection > 1){
            adjustedLeftBackSpeed = 1;
        }else if(motorPowerFactors.get(leftBackDrive)*targetSpeed - powerCorrection < -1) {
            adjustedLeftBackSpeed = -1;
        }else{
            adjustedLeftBackSpeed = motorPowerFactors.get(leftBackDrive) * targetSpeed - powerCorrection;
        }

        if(motorPowerFactors.get(rightFrontDrive)*targetSpeed + powerCorrection > 1){
            adjustedRightFrontSpeed = 1;
        }else if(motorPowerFactors.get(rightFrontDrive)*targetSpeed + powerCorrection < -1) {
            adjustedRightFrontSpeed = -1;
        }else{
            adjustedRightFrontSpeed = motorPowerFactors.get(rightFrontDrive) * targetSpeed + powerCorrection;
        }

        if(motorPowerFactors.get(rightBackDrive)*targetSpeed + powerCorrection > 1){
            adjustedRightBackSpeed = 1;
        }else if(motorPowerFactors.get(rightBackDrive)*targetSpeed + powerCorrection < -1) {
            adjustedRightBackSpeed = -1;
        }else{
            adjustedRightBackSpeed = motorPowerFactors.get(rightBackDrive) * targetSpeed + powerCorrection;
        }

        mPriorImuAngle = mCurrentImuAngle;
        mPriorAdjustedAngle = mAdjustedAngle;
        Log.i(TAG, String.format("getSpeedsPhi: targetSpeed: %.3f, powerCorrection: %.3f", targetSpeed, powerCorrection));
        Log.i(TAG, String.format("getSpeedsPhi: adjustedLeftFrontSpeed: %.3f, adjustedLeftBackSpeed: %.3f, adjustedRightFrontSpeed: %.3f, adjustedRightBackSpeed: %.3f",
                adjustedLeftFrontSpeed, adjustedLeftBackSpeed, adjustedRightFrontSpeed, adjustedRightBackSpeed));
        Log.i(TAG, String.format("getSpeedsPhi: leftFrontMotorPowerFactor: %.3f, leftBackMotorPowerFactor: %.3f, rightFrontMotorPowerFactor: %.3f, rightBackMotorPowerFactor: %.3f",
                motorPowerFactors.get(leftFrontDrive), motorPowerFactors.get(leftBackDrive), motorPowerFactors.get(rightFrontDrive), motorPowerFactors.get(rightBackDrive)));
        return new SpeedsPhi(adjustedLeftFrontSpeed, adjustedLeftBackSpeed, adjustedRightFrontSpeed, adjustedRightBackSpeed);
    }


    /**
     * @return Difference in target angle and nowAngle
     */
    private double calculateAngleDifference(double targetAngle, double nowAngle){
        double angle1, angle2, angleDiff180 = 0, angleDiff0 = 0, angleDifference;
        if(targetAngle >= 0 && nowAngle <= 0){
            angle1 = Math.PI - targetAngle;
            angle2 = nowAngle + Math.PI;
            angleDiff180 = angle1 + angle2;
            angleDiff0 = targetAngle - nowAngle;
            angleDifference = Math.min(angleDiff180, angleDiff0);
            if(angleDiff180 < angleDiff0)
                angleDifference *= -1;
        }else if(targetAngle <= 0 && nowAngle >= 0){
            angle1 = Math.PI - nowAngle;
            angle2 = targetAngle + Math.PI;
            angleDiff180 = angle1 + angle2;
            angleDiff0 = nowAngle - targetAngle;
            angleDifference = Math.min(angleDiff180, angleDiff0);
            if(angleDiff0 < angleDiff180)
                angleDifference *= -1;
        }else{
            // for all cases that target & now have the same sign
            angleDifference = targetAngle - nowAngle;
        }
        Log.i(TAG, String.format("calculateAngleDifference: angleDiff0: %.2f, angleDiff180: %.2f, angleDifference: %.2f",
                angleDiff0, angleDiff180, angleDifference));
        return angleDifference;
    }

    private boolean shouldStopIfApplicable(boolean shouldMonitorAcceleration, long startTimeMillis){
        if (shouldMonitorAcceleration) {
            //android.util.Log.w("#%#%", "WE GOT HERE");

            if ((System.currentTimeMillis() - startTimeMillis) < 400)
                return false; //Wait 1 sec to allow for initial acceleration
            double a = robot.imu.getLinearAcceleration().yAccel;
            double current = 0;
            for (org.firstinspires.ftc.teamcode.robot.hardware.Drive drive : robot.drives.values()) {
                //double c = drive.getCurrent();
                //android.util.Log.w("current", "" + c);
                current += drive.getCurrent();
            }
            android.util.Log.w("isHighAcceleration", "Y Accel: " + a + " Current: " + current);
            /*if (a > 0.5) {
                android.util.Log.w("isHighAcceleration", "High acceleration detected... Crashed into carousel?");
                return true;
            }*/
            if (current > 8) {
                android.util.Log.w("isHighAcceleration", "High current detected... Crashed into carousel?");
                return true;
            }
            //if(robot.imu.getLinearAcceleration().xAccel > 1.0)
            //    return true;
        }
        return false;
    }
}