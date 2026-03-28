package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class rangeRPMtable {
    private static final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
    public static final double MIN = 1.219;
    public static final double MAX = 2.133;
    public static final double OPTIMAL = (MIN + MAX) / 2;

    static {
        table.put(Feet.of(3.0).in(Meters), 1580.0);
        table.put(Feet.of(4.0).in(Meters), 1720.0);
        table.put(Feet.of(5.0).in(Meters), 1830.0);
        table.put(Feet.of(6.0).in(Meters), 1930.0);
       }

    public static double get(double range) {
        return table.get(range);
    }

    public static boolean inRange(double range) {
        return range >= MIN && range <= MAX;
    }
}
