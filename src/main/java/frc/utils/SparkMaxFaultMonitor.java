package frc.utils;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Centralized SparkMax fault and warning monitor.
 * <p>
 * Call {@link #register(String, SparkMax)} once per motor (typically in the
 * subsystem constructor). Then call {@link #checkAll()} from
 * {@code Robot.robotPeriodic()} so every fault and warning is evaluated each
 * cycle and surfaced as a WPILib {@link Alert}.
 * <p>
 * Faults are reported as {@code kError}, warnings as {@code kWarning}.
 */
public final class SparkMaxFaultMonitor {

    /** Internal record for one registered motor and its pre-built Alerts. */
    private static final class Entry {
        final SparkMax motor;

        // Fault alerts  (AlertType.kError)
        final Alert fCan, fMotorType, fFirmware, fSensor, fTemperature, fGateDriver, fEscEeprom, fOther;

        // Warning alerts (AlertType.kWarning)
        final Alert wBrownout, wOvercurrent, wEscEeprom, wExtEeprom, wSensor, wStall, wHasReset, wOther;

        Entry(String name, SparkMax motor) {
            this.motor = motor;

            // Faults
            fCan         = new Alert(name + ": CAN Fault",         AlertType.kError);
            fMotorType   = new Alert(name + ": MotorType Fault",   AlertType.kError);
            fFirmware    = new Alert(name + ": Firmware Fault",    AlertType.kError);
            fSensor      = new Alert(name + ": Sensor Fault",      AlertType.kError);
            fTemperature = new Alert(name + ": Temperature Fault", AlertType.kError);
            fGateDriver  = new Alert(name + ": GateDriver Fault",  AlertType.kError);
            fEscEeprom   = new Alert(name + ": ESC EEPROM Fault",  AlertType.kError);
            fOther       = new Alert(name + ": Other Fault",       AlertType.kError);

            // Warnings
            wBrownout     = new Alert(name + ": Brownout Warning",     AlertType.kWarning);
            wOvercurrent  = new Alert(name + ": Overcurrent Warning",  AlertType.kWarning);
            wEscEeprom    = new Alert(name + ": ESC EEPROM Warning",   AlertType.kWarning);
            wExtEeprom    = new Alert(name + ": Ext EEPROM Warning",   AlertType.kWarning);
            wSensor       = new Alert(name + ": Sensor Warning",       AlertType.kWarning);
            wStall        = new Alert(name + ": Stall Warning",        AlertType.kWarning);
            wHasReset     = new Alert(name + ": HasReset Warning",     AlertType.kWarning);
            wOther        = new Alert(name + ": Other Warning",        AlertType.kWarning);
        }

        void check() {
            Faults f = motor.getFaults();
            fCan.set(f.can);
            fMotorType.set(f.motorType);
            fFirmware.set(f.firmware);
            fSensor.set(f.sensor);
            fTemperature.set(f.temperature);
            fGateDriver.set(f.gateDriver);
            fEscEeprom.set(f.escEeprom);
            fOther.set(f.other);

            Warnings w = motor.getWarnings();
            wBrownout.set(w.brownout);
            wOvercurrent.set(w.overcurrent);
            wEscEeprom.set(w.escEeprom);
            wExtEeprom.set(w.extEeprom);
            wSensor.set(w.sensor);
            wStall.set(w.stall);
            wHasReset.set(w.hasReset);
            wOther.set(w.other);
        }
    }

    private static final List<Entry> entries = new ArrayList<>();

    private SparkMaxFaultMonitor() {} // prevent instantiation

    /**
     * Register a SparkMax motor for fault/warning monitoring.
     *
     * @param name  Human-readable label, e.g. "Hopper Left" or "Shooter Flywheel"
     * @param motor The SparkMax instance to monitor
     */
    public static void register(String name, SparkMax motor) {
        entries.add(new Entry(name, motor));
    }

    /**
     * Register a motor object that may or may not be a SparkMax (e.g. from
     * {@code SwerveMotor.getMotor()}). If the object is not a SparkMax it is
     * silently ignored.
     *
     * @param name  Human-readable label
     * @param motor The motor object to inspect
     */
    public static void register(String name, Object motor) {
        if (motor instanceof SparkMax) {
            entries.add(new Entry(name, (SparkMax) motor));
        }
    }

    /**
     * Check all registered motors for faults and warnings, updating their
     * corresponding Alerts. Call this once per loop from
     * {@code Robot.robotPeriodic()}.
     */
    public static void checkAll() {
        for (Entry e : entries) {
            e.check();
        }
    }
}
