package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private SparkMax shooterMotor;
  private SparkMax feederMotor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController closedLoopController;
  private double rpmSetpoint = 0.0;
  private double feederSpeed = ShooterConstants.feederSpeedDefault;
  private boolean dangerMode = false;
  private Timer timeAtSpeed = new Timer();
  private double kP = ShooterConstants.kPdefault;
  private double kI = ShooterConstants.kIdefault;
  private double kD = ShooterConstants.kDdefault;
  private double kV = ShooterConstants.kVdefault;
  private double kTol = ShooterConstants.PIDToleranceDefault;
  private double kDfilter = ShooterConstants.kDfilterDefault;
  private double kRampRate = ShooterConstants.kRampRateDefault;
  private double kMaxAccel = ShooterConstants.kMaxAccelDefault;
  private double kMaxVelocity = ShooterConstants.kMaxVelocityDefault;
  // Locally-cached sensor/state values to avoid repeated CAN reads every loop
  private double currentRPM = 0.0;
  private ClosedLoopSlot currentSlot; // initialized from hardware in constructor
  private final double spinUpTime = 2.0; // seconds that the shooter must be at the target speed before we consider it
                                         // "ready" to shoot, can be tuned based on how long it takes for the shooter to
                                         // stabilize at the target speed after a change

  public Shooter(int shooterMotorID, int feederMotorID) {
    shooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.apply(SparkMaxConfig.Presets.REV_NEO_550);
    config.inverted(false).idleMode(IdleMode.kCoast);

    shooterMotor.configure(config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    encoder = shooterMotor.getEncoder();

    loadPreferences();

    // Initialize local gain values from defaults and configure controller
    setPIDVT(kP, kI, kD, kV, kTol, kDfilter, kRampRate, kMaxAccel, kMaxVelocity);
    closedLoopController = shooterMotor.getClosedLoopController();

    feederMotor = new SparkMax(feederMotorID, MotorType.kBrushed);
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.inverted(false)
                .idleMode(IdleMode.kBrake)
                 .smartCurrentLimit(20);
    feederMotor.configure(feederConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    rpmSetpoint = 0.0;
    shooterMotor.set(0); // sets it to zero because it is the default
    feederMotor.set(0);

    // Read the actual slot from the controller once at startup so our cached
    // value reflects whatever the SparkMax persisted from a previous session.
    currentSlot = closedLoopController.getSelectedSlot();
  }


  public void setRPMsetpoint(double rpm) {
    rpmSetpoint = MathUtil.clamp(rpm, 0.0, kMaxVelocity);
    closedLoopController.setSetpoint(rpmSetpoint, ControlType.kMAXMotionVelocityControl, currentSlot);
    if(isOpenLoop()) {timeAtSpeed.restart();}
  }

  public void setRPMsetpoint(double rpm, ClosedLoopSlot slot) {
    rpmSetpoint = MathUtil.clamp(rpm, 0.0, kMaxVelocity);
    closedLoopController.setSetpoint(rpmSetpoint, ControlType.kMAXMotionVelocityControl, slot);
    currentSlot = slot;
    if(isOpenLoop()) {timeAtSpeed.restart();}
  }


  /**
   * Configure the SparkMax closed-loop PID and feedforward values.
   * This centralizes configuration so sendable/profile updates and preference
   * loads
   * can reuse the same routine.
   */
  private void setPIDVT(double p, double i, double d, double v, double tol,
                       double dFilter, double rampRate, double maxAccel, double maxVelocity) {
  // store locally
    kP = p;
    kI = i;
    kD = d;
    kV = v;
    kTol = tol;
    kDfilter = dFilter;
    kRampRate = rampRate;
    kMaxAccel = maxAccel;
    kMaxVelocity = maxVelocity;

    setPIDVT();
  }
  
  /**
   * Apply the currently-stored kP/kI/kD/kV/kTol
   * values to the motor controller.
   */
  private void setPIDVT() {
 
    SparkMaxConfig cfg = new SparkMaxConfig();

    // slot zero has PID and feedforward
    ClosedLoopConfig cl0 = new ClosedLoopConfig()
        .pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
        .dFilter(kDfilter, ClosedLoopSlot.kSlot0)
        .apply(new FeedForwardConfig().kV(kV, ClosedLoopSlot.kSlot0))
        .allowedClosedLoopError(kTol, ClosedLoopSlot.kSlot0)
        .apply(new MAXMotionConfig()
            .maxAcceleration(kMaxAccel, ClosedLoopSlot.kSlot0)
            .cruiseVelocity(kMaxVelocity * 1.5, ClosedLoopSlot.kSlot0));
    cfg.apply(cl0);

    // slot one has only feedforward
    ClosedLoopConfig cl1 = new ClosedLoopConfig()
        .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot1)
        .apply(new FeedForwardConfig().kV(kV, ClosedLoopSlot.kSlot1))
        .allowedClosedLoopError(kTol, ClosedLoopSlot.kSlot1);
    cfg.apply(cl1);

    cfg.closedLoopRampRate(kRampRate);

    shooterMotor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // Read velocity exactly once per loop; all other methods use this cached value
    // instead of going back to the CAN bus independently.
    currentRPM = encoder.getVelocity();
  }

  public double getRPM() {
    return currentRPM;
  }

  public double getMotorPctOutput() {
    return shooterMotor.get();
  }

  public boolean isReady() {
    if(isDangerMode()){
      return true;
    }
    if (isPIDEnabled()) {
      return Math.abs(currentRPM - rpmSetpoint) <= kTol;
    }
    if (isOpenLoop()) {
      return timeAtSpeed.hasElapsed(spinUpTime);
    }
    return false; 
  }

  public void enablePID(boolean enable) {
    currentSlot = enable ? ClosedLoopSlot.kSlot0 : ClosedLoopSlot.kSlot1;
    closedLoopController.setIAccum(0.0);
    closedLoopController.setSetpoint(currentRPM, ControlType.kVelocity, currentSlot); // set the current speed as the setpoint so we don't get a sudden change
  }

  public boolean isPIDEnabled() {
    return closedLoopController.getControlType() == ControlType.kVelocity &&
        currentSlot == ClosedLoopSlot.kSlot0;
  }

  public boolean isOpenLoop() {
    return closedLoopController.getControlType() == ControlType.kVelocity &&
        currentSlot == ClosedLoopSlot.kSlot1;
  }

  public boolean isDangerMode() {
    return dangerMode;
  }

  public void setDangerMode(boolean enabled) {
    dangerMode = enabled;
  }

  public void feedForward() {
    feederMotor.set(feederSpeed);
  }

  public void feedBackward() {
    feederMotor.set(-feederSpeed);
  }

  public void setFeederSpeed(double speed) {
    feederSpeed = speed;
  }

  public double getFeederSpeed() {
    return feederSpeed;
  } 

  public void stopFeeder() {
    feederMotor.set(0.0);
  } 

  public void stop() {
    coast();
    stopFeeder();
  }

  private void coast() {
    shooterMotor.disable();
  }


  public void loadPreferences() {
    if (Preferences.containsKey(ShooterConstants.kPkey)) {
      System.out.println("Loading shooter PID values from preferences");
      kP = Preferences.getDouble(ShooterConstants.kPkey, ShooterConstants.kPdefault);
      kI = Preferences.getDouble(ShooterConstants.kIkey, ShooterConstants.kIdefault);
      kD = Preferences.getDouble(ShooterConstants.kDkey, ShooterConstants.kDdefault);
      kV = Preferences.getDouble(ShooterConstants.kVkey, ShooterConstants.kVdefault);
      kDfilter = Preferences.getDouble(ShooterConstants.kDfilterKey, ShooterConstants.kDfilterDefault);
      kRampRate = Preferences.getDouble(ShooterConstants.kRampRateKey, ShooterConstants.kRampRateDefault);
      kMaxAccel = Preferences.getDouble(ShooterConstants.kMaxAccelKey, ShooterConstants.kMaxAccelDefault);
      kMaxVelocity = Preferences.getDouble(ShooterConstants.kMaxVelocityKey, ShooterConstants.kMaxVelocityDefault);
      // Apply loaded gains and tolerance to the motor controller
      double tol = Preferences.getDouble(ShooterConstants.PIDToleranceKey,
          ShooterConstants.PIDToleranceDefault);
      setPIDVT(kP, kI, kD, kV, tol, kDfilter, kRampRate, kMaxAccel, kMaxVelocity);
      
      feederSpeed = Preferences.getDouble(ShooterConstants.feederSpeedKey,
          ShooterConstants.feederSpeedDefault);
    } else {
      System.out.println("No shooter prefs found. Using default values");
    }
  }

  public void savePreferences() {
    System.out.println("Saving shooter PID values to preferences");
    Preferences.setDouble(ShooterConstants.kPkey, kP);
    Preferences.setDouble(ShooterConstants.kIkey, kI);
    Preferences.setDouble(ShooterConstants.kDkey, kD);
    Preferences.setDouble(ShooterConstants.kVkey, kV);
    Preferences.setDouble(ShooterConstants.kDfilterKey, kDfilter);
    Preferences.setDouble(ShooterConstants.kRampRateKey, kRampRate);
    Preferences.setDouble(ShooterConstants.kMaxAccelKey, kMaxAccel);
    Preferences.setDouble(ShooterConstants.kMaxVelocityKey, kMaxVelocity);
    // No direct API to read closed-loop allowed error from the controller here;
    // save the default
    Preferences.setDouble(ShooterConstants.PIDToleranceKey, kTol);
  }

  public Command incrementSpeedCommand() {
    return new RunCommand(() -> {
      setRPMsetpoint(getRPMsetpoint() + ShooterConstants.manualRPMincrement);
    }, this);
  }

  public Command decrementSpeedCommand() {
    return new RunCommand(() -> {
      setRPMsetpoint(getRPMsetpoint() - ShooterConstants.manualRPMincrement);
    }, this);
  }

  public String getStatus(){
    ControlType controlType = closedLoopController.getControlType();
    if(controlType == ControlType.kVelocity){
      if(currentSlot == ClosedLoopSlot.kSlot0){
        return "PID";
      }else{
        return "OPEN LOOP";
      }
    }else{
      return "COAST";
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Expose closed-loop gains and allow updating them by reconfiguring the
    // SparkMax
    builder.addDoubleProperty("kP", () -> kP,
        (x) ->  { if (x != kP) { kP = x; setPIDVT(); } } );
    builder.addDoubleProperty("kI", () -> kI,
        (x) -> { if (x != kI) { kI = x; setPIDVT(); } } );
    builder.addDoubleProperty("kD", () -> kD,
        (x) -> { if (x != kD) { kD = x; setPIDVT(); } } );
    builder.addDoubleProperty("kDfilter", () -> kDfilter,
        (x) -> { kDfilter = MathUtil.clamp(x, 0, 1.0); } );
    builder.addDoubleProperty("kRampRate", () -> kRampRate,
        (x) -> { if (x != kRampRate) { kRampRate = x; setPIDVT(); } });
    builder.addDoubleProperty("kMaxAccel", () -> kMaxAccel,
        (x) -> { if (x != kMaxAccel) { kMaxAccel = x; setPIDVT(); } });
    builder.addDoubleProperty("kMaxVelocity", () -> kMaxVelocity,
        (x) -> { if (x != kMaxVelocity) { kMaxVelocity = x; setPIDVT(); } });
    builder.addDoubleProperty("kV", () -> kV,
        (x) -> { if (x != kV) { kV = x; setPIDVT(); } } );
    builder.addDoubleProperty("kTolerance", () -> kTol,
        (x) -> { if (x != kTol) { kTol = x; setPIDVT(); } } );
    builder.addDoubleProperty("Shooter RPM", () -> getRPM(), null);
    builder.addDoubleProperty("Shooter RPM raw", () -> currentRPM, null);
    builder.addDoubleProperty("Shooter SetpointRPM", () -> getRPMsetpoint(),
        (x) -> setRPMsetpoint(x));
    builder.addBooleanProperty("Ready?", () -> isReady(), null);
    builder.addDoubleProperty("Feeder Speed", () -> feederSpeed,
        (x) -> feederSpeed = x);
    builder.addBooleanProperty("Danger Mode", () -> isDangerMode(), (x) -> setDangerMode(x));
    builder.addBooleanProperty("Save Prefs", () -> false, (x) -> {
      if (x)
        savePreferences();
    });

    builder.addDoubleProperty("shoot motor output", () -> shooterMotor.getAppliedOutput(), null);
    builder.addDoubleProperty ("feed motor output",()->feederMotor.get(), null);
  }

  public double getRPMsetpoint() {
    return rpmSetpoint;
  }

  public Command spinUpCommand(DoubleSupplier rpmSupplier) {
    return new InstantCommand(() -> {
      setRPMsetpoint(rpmSupplier.getAsDouble());
    }, this);
  }

  public Command shootCommand(){
    return new FunctionalCommand(
      ()->{},
      ()->{ if(isReady()){
              feedForward();
            }else{
              stopFeeder();
            }},
      (interrupted)->{stopFeeder();},
      ()->false,
      this);
  }

  public Command reverseFeedCommand(){
    return new FunctionalCommand(
      ()->{},
      ()->{feedBackward();},
      (interrupted)->{stopFeeder();},
      ()->false,
      this);
  }

}