package org.firstinspires.ftc.teamcode.opmode;

public class Configuration {
    // - place pixel?
    // - use inner or outer paths?
    // - park or drop on board?
    //String park = "none";
    boolean placePixel = true;
    boolean doPark = true;
    boolean innerPath = true; // ignored if doPark is false
    boolean dropOnBoard = true; // ignored if doPark is false

    boolean fieldCentric = true;
    boolean tensorFlowInInit = false;
}
