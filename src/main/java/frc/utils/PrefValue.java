package frc.utils;

import java.util.concurrent.CopyOnWriteArrayList;

import edu.wpi.first.wpilibj.Preferences;

public class PrefValue<T> {

    // Central, thread-safe registry of all PrefValue instances.
    private static final CopyOnWriteArrayList<PrefValue<?>> registry = new CopyOnWriteArrayList<>();

    public static void saveAll() {
        for (PrefValue<?> p : registry) {
            p.save();
        }
    }

    public static void loadAll() {
        for (PrefValue<?> p : registry) {
            p.load();
        }
    }

    protected final String key;
    protected final T defaultValue;
    protected T value;

    public PrefValue(String key, T defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        registry.add(this);
        load();
    }

    public String getKey() {
        return key;
    }

    public T getDefaultValue() {
        return defaultValue;
    }

    public T get() {
        return value;
    }

    public void set(T v) {
        this.value = v;
    }

    /**
     * Dispatches load from WPILib Preferences based on the type of defaultValue.
     */
    @SuppressWarnings("unchecked")
    public void load() {
        if (defaultValue instanceof Integer) {
            set((T) (Integer) Preferences.getInt(key, (Integer) defaultValue));
        } else if (defaultValue instanceof Double) {
            set((T) (Double) Preferences.getDouble(key, (Double) defaultValue));
        } else if (defaultValue instanceof Boolean) {
            set((T) (Boolean) Preferences.getBoolean(key, (Boolean) defaultValue));
        } else if (defaultValue instanceof String) {
            set((T) Preferences.getString(key, (String) defaultValue));
        } else {
            throw new IllegalArgumentException(
                "PrefValue: unsupported type for key \"" + key + "\": "
                + defaultValue.getClass().getSimpleName()
                + ". Supported types: Integer, Double, Boolean, String.");
        }
    }

    /**
     * Dispatches save to WPILib Preferences based on the type of the current value.
     */
    public void save() {
        if (value instanceof Integer) {
            Preferences.setInt(key, (Integer) value);
        } else if (value instanceof Double) {
            Preferences.setDouble(key, (Double) value);
        } else if (value instanceof Boolean) {
            Preferences.setBoolean(key, (Boolean) value);
        } else if (value instanceof String) {
            Preferences.setString(key, (String) value);
        } else {
            throw new IllegalArgumentException(
                "PrefValue: unsupported type for key \"" + key + "\": "
                + value.getClass().getSimpleName()
                + ". Supported types: Integer, Double, Boolean, String.");
        }
    }

}
