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

    /**
     * Adapter interface that knows how to load/save a typed value from WPILib Preferences.
     */
    @FunctionalInterface
    public interface PrefTypeAdapter<T> {
        T load(String key, T defaultValue);

        default void save(String key, T value) {
            // Default no-op; adapters that support saving should override.
        }
    }

    // Built-in adapters for primitive-backed Preferences methods
    public static final PrefTypeAdapter<Integer> INT_ADAPTER = new PrefTypeAdapter<Integer>() {
        @Override
        public Integer load(String key, Integer defaultValue) {
            return Preferences.getInt(key, defaultValue);
        }

        @Override
        public void save(String key, Integer value) {
            Preferences.setInt(key, value);
        }
    };

    public static final PrefTypeAdapter<Double> DOUBLE_ADAPTER = new PrefTypeAdapter<Double>() {
        @Override
        public Double load(String key, Double defaultValue) {
            return Preferences.getDouble(key, defaultValue);
        }

        @Override
        public void save(String key, Double value) {
            Preferences.setDouble(key, value);
        }
    };

    public static final PrefTypeAdapter<Boolean> BOOLEAN_ADAPTER = new PrefTypeAdapter<Boolean>() {
        @Override
        public Boolean load(String key, Boolean defaultValue) {
            return Preferences.getBoolean(key, defaultValue);
        }

        @Override
        public void save(String key, Boolean value) {
            Preferences.setBoolean(key, value);
        }
    };

    public static final PrefTypeAdapter<String> STRING_ADAPTER = new PrefTypeAdapter<String>() {
        @Override
        public String load(String key, String defaultValue) {
            return Preferences.getString(key, defaultValue);
        }

        @Override
        public void save(String key, String value) {
            Preferences.setString(key, value);
        }
    };

    

    protected final String key;
    protected final T defaultValue;
    protected T value;
    private final PrefTypeAdapter<T> adapter;

    public PrefValue(String key, T defaultValue, PrefTypeAdapter<T> adapter) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.adapter = adapter;
        // Register centrally so callers can discover prefs at runtime
        registry.add(this);
    }

    /**
     * Convenience constructor: pick a built-in adapter based on the runtime type of defaultValue.
     */
    @SuppressWarnings("unchecked")
    public PrefValue(String key, T defaultValue) {
        this(
            key,
            defaultValue,
            (PrefTypeAdapter<T>) (
                (defaultValue instanceof Integer) ? INT_ADAPTER
                : (defaultValue instanceof Double) ? DOUBLE_ADAPTER
                : (defaultValue instanceof Boolean) ? BOOLEAN_ADAPTER
                : (defaultValue instanceof String) ? STRING_ADAPTER
                : null
            )
        );
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
     * Load the value from persistent storage using the adapter.
     */
    public void load() {
        if (adapter != null) {
            set(adapter.load(key, defaultValue));
        }
    }

    /**
     * Save the value to persistent storage using the adapter.
     */
    public void save() {
        if (adapter != null) {
            adapter.save(key, get());
        }
    }

}

