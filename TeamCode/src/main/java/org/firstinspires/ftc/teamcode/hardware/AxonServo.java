package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.util.Timeout;

public class AxonServo {
    public CRServo innerServo;
    public AnalogInput innerAnalog;

    // Create a new background thread to count rotations
    private final ServoCounter counter;

    private class ServoCounter extends Thread {
        private int count = 0;

        public void run() {
            android.util.Log.w("AXON_THREAD", "Starting thread");
            double lastPosition = getCurrentPosition();
            while (!GlobalOpMode.opMode.isStopRequested()) {
                //android.util.Log.w("AXON_THREAD", "LOOP");
                Thread.yield();
                double currentPosition = getCurrentPosition();
                // Positive wrap-around from 0 -> 360
                if (currentPosition > 180 && lastPosition < 180 && innerServo.getPower() > 0) {
                    count--;
                }
                // Negative wrap-around from 360 -> 0
                if (currentPosition < 180 && lastPosition > 180 && innerServo.getPower() < 0) {
                    count++;
                }
                lastPosition = currentPosition;
            }
        }

        public int getCount() {
            return count;
        }
    }

    public AxonServo(String servoName, String analogName, HardwareMap hwMap) {
        this.innerServo = hwMap.get(CRServo.class, servoName);
        this.innerAnalog = hwMap.get(AnalogInput.class, analogName);

        counter = new ServoCounter();
        counter.start();
    }

    public double getCurrentPosition() {
        return this.innerAnalog.getVoltage()/3.3 * 360;
    }

    public double getAdjustedPosition() {
        return getCurrentPosition() + counter.getCount() * 360;
    }

    public void setAdjustedPosition(double position, double power) {
        Timeout timeout = new Timeout(5); // 5 sec max on all operations
        if (getAdjustedPosition() < position) {
            this.innerServo.setPower(-power);
            while (getAdjustedPosition() < position && !GlobalOpMode.opMode.isStopRequested() && !timeout.expired()) {
                //GlobalOpMode.opMode.telemetry.addData("Extender Axon Servo Position", getAdjustedPosition());
                //GlobalOpMode.opMode.telemetry.update();
                //GlobalOpMode.opMode.sleep(1);
            }
        } else if (getAdjustedPosition() > position) {
            this.innerServo.setPower(power);
            while (getAdjustedPosition() > position && !GlobalOpMode.opMode.isStopRequested() && !timeout.expired()) {
                //GlobalOpMode.opMode.telemetry.addData("Extender Axon Servo Position", getAdjustedPosition());
                //GlobalOpMode.opMode.telemetry.update();
                //GlobalOpMode.opMode.sleep(1);
            }
        }
        this.innerServo.setPower(0);
        // Override the current position
    }



}
