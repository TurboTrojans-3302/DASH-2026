package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class rangeRPMtable {
    private final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
    public final double MIN = 1.219;
    public final double MAX = 2.133;

    rangeRPMtable() {
        table.put(1.219, 1550.0);
        table.put(1.524, 1550.0);
        table.put(1.828, 1675.0);
        table.put(2.133, 1700.0);
    }

    public double get(double range) {
        return table.get(range);
    }

    public boolean inRange(double range) {
        return range >= MIN && range <= MAX;
    }
}
