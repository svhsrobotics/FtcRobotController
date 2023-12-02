package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class AxonServo {
    public CRServo innerServo;
    public AnalogInput innerAnalog;

    public AxonServo(String servoName, String analogName, HardwareMap hwMap) {
        this.innerServo = hwMap.get(CRServo.class, servoName);
        this.innerAnalog = hwMap.get(AnalogInput.class, analogName);
    }

    public double getCurrentPosition() {
        return this.innerAnalog.getVoltage()/3.3 *360;
    }



}
