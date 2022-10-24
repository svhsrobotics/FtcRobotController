package org.firstinspires.ftc.teamcode.util;

/**
 * A basic timeout class
 */
public class Timeout {
    private final long startTime;
    private final int duration;

    /**
     * Creates a new timeout
     * @param duration The duration of the timeout in seconds
     */
    public Timeout(int duration) {
        this.duration = duration;
        this.startTime = System.nanoTime();
    }

    /**
     * Converts seconds to nanoseconds
     * @param seconds The number of seconds
     * @return The number of nanoseconds
     */
    private long secondsToNano(int seconds) {
        return (long) seconds * 1000 * 1000 * 1000;
    }

    /**
     * @return false if the timeout has expired
     */
    // I really think that using !expired is more intuitive, so I'm suppressing the warning
    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public boolean expired() {
        return (System.nanoTime() - this.startTime) > secondsToNano(this.duration);
    }
}
