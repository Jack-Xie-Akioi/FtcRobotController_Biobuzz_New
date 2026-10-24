package org.firstinspires.ftc.teamcode.util;

import java.util.Map;
import java.util.TreeMap;

public final class LUT {

    public static final class ShotSettings {

        public final double velocity;
        public final double hoodPosition;

        public ShotSettings(double velocity, double hoodPosition) {
            if (!isFinite(velocity) || velocity <= 0) {
                throw new IllegalArgumentException("VELOCITY MUST BE FINITE AND POSITIVE");
            }

            if (!isFinite(hoodPosition) || hoodPosition < 0 || hoodPosition > 1) {
                throw new IllegalArgumentException("HOOD POSITION MUST BE BETWEEN 0 AND 1");
            }

            this.velocity = velocity;
            this.hoodPosition = hoodPosition;
        }
    }

    private final TreeMap<Double, ShotSettings> points = new TreeMap<>();

    // Calibration table.
    public LUT() {
        // Add measured points here:
        // addPoint(distance, velocityTicksPerSecond, hoodPosition);

    }

    public void addPoint(double distance, double velocity, double hoodPosition) {
        if (!isFinite(distance) || distance < 0) {
            throw new IllegalArgumentException("DISTANCE MUST BE FINITE AND NONNEGATIVE");
        }

        points.put(distance, new ShotSettings(velocity, hoodPosition));
    }

    public boolean isInRange(double distance) {
        return isFinite(distance) && !points.isEmpty() && distance >= points.firstKey() && distance <= points.lastKey();
    }

    public ShotSettings getShot(double distance) {
        if (!isInRange(distance)) {
            return null;
        }

        Map.Entry<Double, ShotSettings> lower = points.floorEntry(distance);

        Map.Entry<Double, ShotSettings> upper = points.ceilingEntry(distance);

        // Exact calibration point.
        if (Double.compare(lower.getKey(), upper.getKey()) == 0) {
            return lower.getValue();
        }

        double fraction = (distance - lower.getKey()) / (upper.getKey() - lower.getKey());

        ShotSettings lowerShot = lower.getValue();
        ShotSettings upperShot = upper.getValue();

        double velocity = lowerShot.velocity + fraction * (upperShot.velocity - lowerShot.velocity);

        double hoodPosition = lowerShot.hoodPosition + fraction * (upperShot.hoodPosition - lowerShot.hoodPosition);

        return new ShotSettings(velocity, hoodPosition);
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}