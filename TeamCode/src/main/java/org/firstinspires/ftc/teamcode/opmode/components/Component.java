package org.firstinspires.ftc.teamcode.opmode.components;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.Robot;

public abstract class Component {
    static Vector2d BLUE_BOARD_CENTER_LINE = new Vector2d(12, 24.5);
    static Vector2d BLUE_BOARD_LEFT_LINE = new Vector2d(23, 30);
    static Vector2d BLUE_BOARD_RIGHT_LINE = new Vector2d(1, 30);
    static Vector2d RED_BOARD_CENTER_LINE = new Vector2d(12, -24.5);
    // LEFT MISSING
    static Vector2d RED_BOARD_RIGHT_LINE = new Vector2d(23, -30);
    //static Vector2d RED_AUDIENCE_CENTER_LINE = new Vector2d(12, -24.5);

    private final Robot robot;

    protected Component(Robot robot) {
        this.robot = robot;
    }

    protected Robot getRobot() {
        return robot;
    }

    public abstract void drive();

}
