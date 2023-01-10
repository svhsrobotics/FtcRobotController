package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.Range;

public class PIController {
    static final double I_WINDUP_LIMIT = 3.0;

    double pGain;
    double iGain;

    double iSum = 0;

    public PIController(double pGain, double iGain) {
        this.pGain = pGain;
        this.iGain = iGain;
    }

    public double update(double input) {
        iSum += input;
        iSum = Range.clip(iSum, -I_WINDUP_LIMIT, I_WINDUP_LIMIT);

        return pGain * input + iGain * iSum;
    }
}
