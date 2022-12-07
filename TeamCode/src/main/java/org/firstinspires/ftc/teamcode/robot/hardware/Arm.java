package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Arm Component. Currently uses hard-coded calibration data.
 * The Arm consists of 3 major components:
 * 1. The Lift, which is 3 motors attached to a vertical linear slide
 * 2. The Reacher, which is a motor attached to a horizontal linear slide
 * 3. The Pincher, which is a servo attached to a custom mechanism that lowers into the cone and expands
 */
public class Arm {
    public Lift lift;
    public Reacher reacher;
    public Pincher pincher;

    public static class Lift {
        private final DcMotor motor1;
        private final DcMotor motor2;
        private final DcMotor motor3;

        public Lift(DcMotor motor1, DcMotor motor2, DcMotor motor3) {
            this.motor1 = motor1;
            this.motor2 = motor2;
            this.motor3 = motor3;

            this.motor1.setDirection(DcMotor.Direction.REVERSE);
            this.motor3.setDirection(DcMotor.Direction.REVERSE);
        }

        public void setPower(double power) {
            // TODO: Enforce limits
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motor1.setPower(power);
            motor2.setPower(power);
            motor3.setPower(power);
        }

        public void setTargetPosition(int position) {
            motor1.setTargetPosition(position);
            motor2.setTargetPosition(position);
            motor3.setTargetPosition(position);

            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motor1.setPower(1); // TODO: Don't hardcode motor power
            motor2.setPower(1);
            motor3.setPower(1);
        }

        public int getTargetPosition() {
            return motor3.getTargetPosition();
        }

        public int getCurrentPosition() {
            return motor3.getCurrentPosition();
        }

        public boolean isBusy() {
            return motor1.isBusy() || motor2.isBusy() || motor3.isBusy();
        }

        public enum Preset {
            GRAB (40),
            GROUND (60),
            DRIVING (400),
            LOW_POLE (1130),
            MEDIUM_POLE (1910),
            HIGH_POLE (2635);

            final int height;
            Preset(int height) {
                this.height = height;
            }
        }

        public void setPreset(Preset preset) {
            setTargetPosition(preset.height);
        }
    }

    public static class Reacher {
        private final DcMotor motor;

        public Reacher(DcMotor motor) {
            this.motor = motor;
            this.motor.setDirection(DcMotor.Direction.REVERSE);
        }

        public void setPower(double power) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(power);
        }

        public void setTargetPosition(int position) {
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1); // TODO: Don't hardcode motor power
        }

        public int getTargetPosition() {
            return motor.getTargetPosition();
        }

        public int getCurrentPosition() {
            return motor.getCurrentPosition();
        }

        public boolean isBusy() {
            return motor.isBusy();
        }
    }

    public static class Pincher {
        private final Servo servo;

        public Pincher(Servo servo) {
            this.servo = servo;
        }

        public void expand() {
            servo.setPosition(0.45);
        }

        public void contract() {
            servo.setPosition(0);
        }
    }

    public Arm(Lift lift, Reacher reacher, Pincher pincher) {
        this.lift = lift;
        this.reacher = reacher;
        this.pincher = pincher;
    }
}
