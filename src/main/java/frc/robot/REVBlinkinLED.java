package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class REVBlinkinLED {

  /* Rev Robotics Blinkin takes a PWM signal from 1000-2000us
   * This is identical to a SparkMax motor. 
   *  -1  corresponds to 1000us
   *  0   corresponds to 1500us
   *  +1  corresponds to 2000us
   */
  private static Spark m_blinkin = null;

  /**
   * Creates a new Blinkin LED controller.
   * 
   * @param pwmPort  The PWM port the Blinkin is connected to.
   */
  public REVBlinkinLED(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
  }

  /*
   * Set the color and blink pattern of the LED strip.
   * 
   * Consult the Rev Robotics Blinkin manual Table 5 for a mapping of values to patterns.
   * 
   * @param val The LED blink color and patern value [-1,1]
   * 
   */ 
  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  public class Pattern {
    public static final double RAINBOW_RAINBOW_PALETTE = -0.99;
    public static final double RAINBOW_PARTY_PALETTE = -0.97;
    public static final double RAINBOW_OCEAN_PALETTE = -0.95;
    public static final double RAINBOW_LAVE_PALETTE = -0.93;
    public static final double RAINBOW_FOREST_PALETTE = -0.91;
    public static final double RAINBOW_WITH_GLITTER = -0.89;
    public static final double CONFETTI = -0.87;
    public static final double SHOT_RED = -0.85;
    public static final double SHOT_BLUE = -0.83;
    public static final double SHOT_WHITE = -0.81;
    public static final double SINELON_RAINBOW_PALETTE = -0.79;
    public static final double SINELON_PARTY_PALETTE = -0.77;
    public static final double SINELON_OCEAN_PALETTE = -0.75;
    public static final double SINELON_LAVA_PALETTE = -0.73;
    public static final double SINELON_FOREST_PALETTE = -0.71;
    public static final double BEATS_PER_MINUTE_RAINBOW_PALETTE = -0.69;
    public static final double BEATS_PER_MINUTE_PARTY_PALETTE = -0.67;
    public static final double BEATS_PER_MINUTE_OCEAN_PALETTE = -0.65;
    public static final double BEATS_PER_MINUTE_LAVA_PALETTE = -0.63;
    public static final double BEATS_PER_MINUTE_FOREST_PALETTE = -0.61;
    public static final double FIRE_MEDIUM = -0.59;
    public static final double FIRE_LARGE = -0.57;
    public static final double TWINKLES_RAINBOW_PALETTE = -0.55;
    public static final double TWINKLES_PARTY_PALETTE = -0.53;
    public static final double TWINKLES_OCEAN_PALETTE = -0.51;
    public static final double TWINKLES_LAVA_PALETTE = -0.49;
    public static final double TWINKLES_FOREST_PALETTE = -0.47;
    public static final double COLOR_WAVES_RAINBOW_PALETTE = -0.45;
    public static final double COLOR_WAVES_PARTY_PALETTE = -0.43;
    public static final double COLOR_WAVES_OCEAN_PALETTE = -0.41;
    public static final double COLOR_WAVES_LAVA_PALETTE = -0.39;
    public static final double COLOR_WAVES_FOREST_PALETTE = -0.37;
    public static final double LARSON_SCANNER_RED = -0.35;
    public static final double LARSON_SCANNER_GRAY = -0.33;
    public static final double LIGHT_CHASE_RED = -0.31;
    public static final double LIGHT_CHASE_BLUE = -0.29;
    public static final double LIGHT_CHASE_GRAY = -0.27;
    public static final double HEARTBEAT_RED = -0.25;
    public static final double HEARTBEAT_BLUE = -0.23;
    public static final double HEARTBEAT_WHITE = -0.21;
    public static final double HEARTBEAT_GRAY = -0.19;
    public static final double BREATH_RED = -0.17;
    public static final double BREATH_BLUE = -0.15;
    public static final double BREATH_GRAY = -0.13;
    public static final double STROBE_RED = -0.11;
    public static final double STROBE_BLUE = -0.09;
    public static final double STROBE_GOLD = -0.07;
    public static final double STROBE_WHITE = -0.05;
    public static final double COLOR1_END_TO_END_BLEND_TO_BLACK = -0.03;
    public static final double COLOR1_LARSON_SCANNER = -0.01;
    public static final double COLOR1_LIGHT_CHASE = 0.01;
    public static final double COLOR1_HEARTBEAT_SLOW = 0.03;
    public static final double COLOR1_HEARTBEAT_MEDIUM = 0.05;
    public static final double COLOR1_HEARTBEAT_FAST = 0.07;
    public static final double COLOR1_BREATH_SLOW = 0.09;
    public static final double COLOR1_BREATH_FAST = 0.11;
    public static final double COLOR1_SHOT = 0.13;
    public static final double COLOR1_STROBE = 0.15;
    public static final double COLOR2_END_TO_END_BLEND_TO_BLACK = 0.17;
    public static final double COLOR2_LARSON_SCANNER = 0.19;
    public static final double COLOR2_LIGHT_CHASE = 0.21;
    public static final double COLOR2_HEARTBEAT_SLOW = 0.23;
    public static final double COLOR2_HEARTBEAT_MEDIUM = 0.25;
    public static final double COLOR2_HEARTBEAT_FAST = 0.27;
    public static final double COLOR2_BREATH_SLOW = 0.29;
    public static final double COLOR2_BREATH_FAST = 0.31;
    public static final double COLOR2_SHOT = 0.33;
    public static final double COLOR2_STROBE = 0.35;
    public static final double COLOR1_AND_2_SPARKLE_COLOR1_ON_COLOR_2 = 0.37;
    public static final double COLOR1_AND_2_SPARKLE_COLOR2_ON_COLOR_1 = 0.39;
    public static final double COLOR1_AND_2_COLOR_GRADIENT_COLOR1_AND_2 = 0.41;
    public static final double COLOR1_AND_2_BEATS_PER_MINUTE_COLOR1_AND_2 = 0.43;
    public static final double COLOR1_AND_2_END_TO_END_BLEND_COLOR1_TO_2 = 0.45;
    public static final double COLOR1_AND_2_END_TO_END_BLEND = 0.47;
    public static final double COLOR1_AND_2_COLOR1_AND_COLOR2_NO_BLENDING = 0.49; 
    public static final double COLOR1_AND_2_TWINKLES_COLOR1_AND_2 = 0.51;
    public static final double COLOR1_AND_2_COLOR_WAVES_COLOR1_AND_2 = 0.53;
    public static final double COLOR1_AND_2_SINELON_COLOR1_AND_2 = 0.55;
    public static final double SOLID_HOT_PINK = 0.57;
    public static final double SOLID_DARK_RED = 0.59;
    public static final double SOLID_RED = 0.61;
    public static final double SOLID_RED_ORANGE = 0.63;
    public static final double SOLID_ORANGE = 0.65;
    public static final double SOLID_GOLD = 0.67;
    public static final double SOLID_YELLOW = 0.69;
    public static final double SOLID_LAWN_GREEN = 0.71;
    public static final double SOLID_LIME = 0.73;
    public static final double SOLID_DARK_GREEN = 0.75;
    public static final double SOLID_GREEN = 0.77;
    public static final double SOLID_BLUE_GREEN = 0.79;
    public static final double SOLID_AQUA = 0.81;
    public static final double SOLID_SKY_BLUE = 0.83;
    public static final double SOLID_DARK_BLUE = 0.85;
    public static final double SOLID_BLUE = 0.87;
    public static final double SOLID_BLUE_VIOLET = 0.89;
    public static final double SOLID_VIOLET = 0.91;
    public static final double SOLID_WHITE = 0.93;
    public static final double SOLID_GRAY = 0.95;
    public static final double SOLID_DARK_GRAY = 0.97;
    public static final double SOLID_BLACK = 0.99;    
  }
  
}