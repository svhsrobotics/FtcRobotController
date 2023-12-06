package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class AxonServo {
    public CRServo innerServo;
    public AnalogInput innerAnalog;

    // Create a new background thread to count rotations
    private final ServoCounter counter;

    private class ServoCounter extends Thread {
        private int count = 0;

        public void run() {
            double lastPosition = getCurrentPosition();
            while (true) {
                android.util.Log.w("AXON_THREAD", "LOOP");
                double currentPosition = getCurrentPosition();
                // Negative wrap-around from 360 -> 0
                if (currentPosition > 180 && lastPosition < 180 && innerServo.getPower() > 0) {
                    count--;
                }
                // Positive wrap-around from 0 -> 360
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



}
