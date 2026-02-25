package frc.utils;

public class Preference {


    public Class IntegerPref extends Integer {
        private static HashSet<IntegerPref> all = new HashSet<IntegerPref>();

        private String key;
        private final Integer defaultValue;

        public IntegerPref(String key, Integer defaultValue){
            super(defaultValue);
            this.key = key;
            this.defaultValue = defaultValue;
            all.add(this);
        }
    

        public IntegerPref load(){
            this = Preferences.getInteger(key, defaultValue)
            return this;
        }

        public IntegerPref save(){
            Preferences.setInteger(key, this);
        }

        public static void saveAll(){
            all.foreach(inst -> inst.save());
        }

        public static void loadAll(){
            all.foreach(inst -> inst.load());
        }

    }
}