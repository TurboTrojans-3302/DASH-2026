package frc.utils;

import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
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

    public static void addAllBuilderProperties(Sendable parentObject, SendableBuilder builder) {
        for (PrefValue<?> p : registry) {
            if (p.parentObject == parentObject) {
                p.addBuilderProperty(builder);
            }
        }
    }

    public static void saveObjectPrefs(Sendable parentObject) {
        System.out.println("Saving preferences for " + parentObject.getClass().getSimpleName());
    
        for (PrefValue<?> p : registry) {
            if (p.parentObject == parentObject) {
                p.save();
            }
        }
    }

    public static void reLoadObjectPrefs(Sendable parentObject) {
        System.out.println("Loading preferences for " + parentObject.getClass().getSimpleName());
    
        for (PrefValue<?> p : registry) {
            if (p.parentObject == parentObject) {
                p.load();
            }
        }
    }

    protected final String key;
    protected final T defaultValue;
    protected T value;
    protected Object parentObject;
    protected Consumer<T> onChange;

    public PrefValue(String key, T defaultValue, Object parentObject) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.parentObject = parentObject;
        registry.add(this);
        load();
    }

    public PrefValue(String key, T defaultValue) {
        this(key, defaultValue, null);
    }

    /**
     * Sets a callback that fires whenever set() is called with a new value.
     * Returns this for chaining.
     */
    public PrefValue<T> onChange(Consumer<T> callback) {
        this.onChange = callback;
        return this;
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
        boolean changed = (this.value == null) ? (v != null) : !this.value.equals(v);
        this.value = v;
        if (changed && onChange != null) {
            onChange.accept(v);
        }
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

    @SuppressWarnings("unchecked")
    public void addBuilderProperty(SendableBuilder builder) {
        if (defaultValue instanceof Integer) {
            builder.addIntegerProperty(key, () -> (Integer) get(), v -> set((T) Integer.valueOf((int) v)));
        } else if (defaultValue instanceof Double) {
            builder.addDoubleProperty(key, () -> (Double) get(), v -> set((T) Double.valueOf(v)));
        } else if (defaultValue instanceof Boolean) {
            builder.addBooleanProperty(key, () -> (Boolean) get(), v -> set((T) Boolean.valueOf(v)));
        } else if (defaultValue instanceof String) {
            builder.addStringProperty(key, () -> (String) get(), v -> set((T) v));
        } else {
            throw new IllegalArgumentException(
                "PrefValue: unsupported type for key \"" + key + "\": "
                + defaultValue.getClass().getSimpleName()
                + ". Supported types: Integer, Double, Boolean, String.");
        }
    }

}
