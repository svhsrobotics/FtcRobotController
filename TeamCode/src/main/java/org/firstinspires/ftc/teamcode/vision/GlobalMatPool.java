package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * A pool of Mats that can be reused to avoid memory leaks.
 * This pool is designed to be used globally, and is not thread-safe,
 * and can only be used by a single Pipeline at a time.
 */
public class GlobalMatPool {
    private static final ConcurrentLinkedQueue<Mat> pool = new ConcurrentLinkedQueue<>();
    private static final ConcurrentLinkedQueue<Mat> inUse = new ConcurrentLinkedQueue<>();

    /**
     * Pulls a Mat from the pool, or creates a new one if the pool is empty.
     * Note: The Mat is not guaranteed to be empty.
     * @return A Mat
     */
    public static Mat get() {
        // Pull a mat from the pool
        Mat mat = pool.poll();
        // If the pool is empty, create a new Mat
        if (mat == null) {
            mat = new Mat();
        }
        inUse.add(mat);
        return mat;
    }

    public static int getUsedMatCount() {
        return inUse.size();
    }

    /**
     * Returns all Mats that are currently in use to the pool.
     * This does not prevent existing references from using the Mats: this method should be called at the end of the pipeline.
     * Note: This does not clear the Mats, so they will still contain the data they had before.
     */
    public static void returnAll() {
        pool.addAll(inUse);
        inUse.clear();
    }

    /**
     * Wrapper around {@link #returnAll()} that also prints debug information to the telemetry.
     * @param telemetry The telemetry to print to
     */
    public static void returnAll(Telemetry telemetry) {
        telemetry.addData("MatPool", "Returned " + inUse.size() + " Mats");
        returnAll();
    }

}
