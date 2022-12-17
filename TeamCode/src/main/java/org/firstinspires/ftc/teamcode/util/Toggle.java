package org.firstinspires.ftc.teamcode.util;

/**
 * This is a simple class that allows you to implement a toggle button.
 * Simply call update() with the button's current value and it will return the toggle's state.
 * You can also read and set state directly to override it.
 */
public class Toggle {
    public boolean state = false;
    private boolean lastState = false;

    public boolean update(boolean buttonState) {
        // If the button is pressed and our last state was not pressed
        if (buttonState && !lastState) {
            // Toggle our current state
            state = !state;
        }

        // Save the state
        lastState = buttonState;

        return state;
    }
}
