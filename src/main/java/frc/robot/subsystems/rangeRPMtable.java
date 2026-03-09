package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class rangeRPMtable {
    private static final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
    public static final double MIN = 1.219;
    public static final double MAX = 2.133;
    public static final double OPTIMAL = (MIN + MAX) / 2;

    static {
        table.put(1.219, 1550.0);
        table.put(1.524, 1550.0);
        table.put(1.828, 1675.0);
        table.put(2.133, 1700.0);
    }

    public static double get(double range) {
        return table.get(range);
    }

    public static boolean inRange(double range) {
        return range >= MIN && range <= MAX;
    }
}
