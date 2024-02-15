package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.GlobalOpMode;
import org.firstinspires.ftc.teamcode.util.Timeout;

import java.util.ArrayList;

public class AxonServo {
    public CRServo innerServo;
    public AnalogInput innerAnalog = null;

    // Create a new background thread to count rotations
    private int count = 0;
    //private final ServoCounter counter;

//    private class ServoCounter extends Thread {
//        private int count = 0;



//        public void run() {
//            try {
//                android.util.Log.w("AXON_THREAD", "Starting thread");
//                double lastPosition = getCurrentPosition();
//                while (!GlobalOpMode.opMode.isStopRequested()) {
//                    //android.util.Log.w("AXON_THREAD", "LOOP");
//                    Thread.yield();
//                    double currentPosition = getCurrentPosition();
//                    // Positive wrap-around from 0 -> 360
//                    if (currentPosition > 180 && lastPosition < 180 && innerServo.getPower() > 0) {
//                        count--;
//                    }
//                    // Negative wrap-around from 360 -> 0
//                    if (currentPosition < 180 && lastPosition > 180 && innerServo.getPower() < 0) {
//                        count++;
//                    }
//                    lastPosition = currentPosition;
//                }
//            } catch (Exception e) {
//                android.util.Log.e("AXON_THREAD", "Crashed with error: " + e);
//            }
//        }
//
//        public int getCount() {
//            return count;
//        }
//    }

    private double lastPosition = -999999999;
    private void update() {
        if (this.innerAnalog == null) {
            return;
        }
        if (lastPosition == -999999999) {
            lastPosition = getCurrentPosition();
        }
        //double lastPosition = getCurrentPosition();

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

    private static final ArrayList<AxonServo> allServos = new ArrayList<>();

    public static void updateAll() {
        for (AxonServo servo : allServos) {
            servo.update();
        }
    }

    public AxonServo(String servoName, String analogName, HardwareMap hwMap) {
        this.innerServo = hwMap.get(CRServo.class, servoName);
        if (analogName != null) {
            this.innerAnalog = hwMap.get(AnalogInput.class, analogName);
            allServos.add(this);
        }

        //counter = new ServoCounter();
        //counter.start();
    }

    private double getCurrentPosition() {
        if (this.innerAnalog == null) {

        }
        return this.innerAnalog.getVoltage()/3.3 * 360;
    }

    public double getAdjustedPosition() {
        return getCurrentPosition() + this.count * 360;
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
