package frc.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Preferences;

public class Prefs {

    public static void saveAll() {
        PrefInteger.all.forEach(p -> p.save());
        PrefDouble.all.forEach(p -> p.save());
        PrefBoolean.all.forEach(p -> p.save());
        PrefString.all.forEach(p -> p.save());
    }

    public static void loadAll() {
        PrefInteger.all.forEach(p -> p.load());
        PrefDouble.all.forEach(p -> p.load());
        PrefBoolean.all.forEach(p -> p.load());
        PrefString.all.forEach(p -> p.load());
    }

    public static class PrefValue<T> {
        protected String key;
        protected T defaultValue;
        protected T value;

        public PrefValue(String key, T defaultValue) {
            this.key = key;
            this.defaultValue = defaultValue;
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
    }

    public static class PrefInteger extends PrefValue<Integer> {

        public static List<PrefInteger> all = new ArrayList<>();

        public PrefInteger(String key, Integer defaultValue) {
            super(key, defaultValue);
            load();
            all.add(this);
        }

        public Integer get() {
            return value;
        }

        public void set(Integer v) {
            super.set(v);
        }

        public void load() {
            set(Preferences.getInt(this.key, defaultValue));
        }

        public void save() {
            Preferences.setInt(this.key, get());
        }
    }

    public static class PrefDouble extends PrefValue<Double> {
        public static List<PrefDouble> all = new ArrayList<>();

        public PrefDouble(String key, Double defaultValue) {
            super(key, defaultValue);
            load();
            all.add(this);
        }

        public Double get() {
            return value;
        }

        public void set(Double v) {
            super.set(v);
        }

        public void load() {
            set(Preferences.getDouble(this.key, defaultValue));
        }

        public void save() {
            Preferences.setDouble(this.key, get());
        }
    }

    public static class PrefBoolean extends PrefValue<Boolean> {
        public static List<PrefBoolean> all = new ArrayList<>();

        public PrefBoolean(String key, Boolean defaultValue) {
            super(key, defaultValue);
            load();
            all.add(this);
        }

        public Boolean get() {
            return value;
        }

        public void set(Boolean v) {
            super.set(v);
        }

        public void load() {
            set(Preferences.getBoolean(this.key, defaultValue));
        }

        public void save() {
            Preferences.setBoolean(this.key, get());
        }
    }

    public static class PrefString extends PrefValue<String> {
        public static List<PrefString> all = new ArrayList<>();

        public PrefString(String key, String defaultValue) {
            super(key, defaultValue);
            load();
            all.add(this);
        }

        public String get() {
            return value;
        }

        public void set(String v) {
            super.set(v);
        }

        public void load() {
            set(Preferences.getString(this.key, defaultValue));
        }

        public void save() {
            Preferences.setString(this.key, get());
        }
    }
}