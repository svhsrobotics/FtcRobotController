package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;

public class AxonServo {
    public Servo innerServo;
    public AnalogInput innerAnalog;

    public AxonServo(String servoName, String analogName, HardwareMap hwMap) {
        this.innerServo = hwMap.get(Servo.class, servoName);
        this.innerAnalog = hwMap.get(AnalogInput.class, analogName);
    }

    public double getCurrentPosition() {
        return this.innerAnalog.getVoltage()/3.3 *360;
    }


}
