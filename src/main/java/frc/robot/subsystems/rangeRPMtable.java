package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class rangeRPMtable {
    private static final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
    public static final double MIN = 1.219;
    public static final double MAX = 2.133;
    public static final double OPTIMAL = (MIN + MAX) / 2;

    static {
        table.put(0.6096, 1775.0);
        table.put(0.9144, 1800.0);
        table.put(1.2192, 1880.0);
        table.put(1.524, 1910.0);
        table.put(1.8288, 2130.0);
    }

    public static double get(double range) {
        return table.get(range);
    }

    public static boolean inRange(double range) {
        return range >= MIN && range <= MAX;
    }
}
