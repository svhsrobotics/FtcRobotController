package org.firstinspires.ftc.teamcode.robot.hardware;

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
        public Lift() {

        }

        // This is just one possible implementation, "target" based

        public void setTargetPosition(int position) {

        }

        public int getTargetPosition() {
            return 0;
        }

        public int getCurrentPosition() {
            return 0;
        }

        public boolean isBusy() {
            return false;
        }
    }

    public static class Reacher {
        public Reacher() {

        }

        // This is just one possible implementation, "target" based

        public void setTargetPosition(int position) {

        }

        public int getTargetPosition() {
            return 0;
        }

        public int getCurrentPosition() {
            return 0;
        }

        public boolean isBusy() {
            return false;
        }
    }

    public static class Pincher {
        public Pincher() {

        }

        public void expand() {

        }

        public void contract() {

        }
    }

    public Arm(Lift lift, Reacher reacher, Pincher pincher) {
        this.lift = lift;
        this.reacher = reacher;
        this.pincher = pincher;
    }

    public enum Preset {

    }

    public void goToPreset(Preset preset) {

    }
}
