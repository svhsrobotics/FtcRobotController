package org.firstinspires.ftc.teamcode.opmode.components;

import org.firstinspires.ftc.teamcode.drive.Robot;

public abstract class Component {

    private final Robot robot;

    protected Component(Robot robot) {
        this.robot = robot;
    }

    protected Robot getRobot() {
        return robot;
    }

    public abstract void drive();

}
