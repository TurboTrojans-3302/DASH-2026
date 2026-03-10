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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.PrefValue;

public class Shooter extends SubsystemBase {
  private SparkMax shooterMotor;
  private SparkMax feederMotor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController closedLoopController;
  private double rpmSetpoint = 0.0;
  private boolean dangerMode = false;
  private Timer timeAtSpeed = new Timer();

  // Tunable parameters backed by WPILib Preferences
  private final PrefValue<Double> kP        = new PrefValue<>("shooter_kP",           0.001,   this);
  private final PrefValue<Double> kI        = new PrefValue<>("shooter_kI",           0.0,     this);
  private final PrefValue<Double> kD        = new PrefValue<>("shooter_kD",           0.0002,  this);
  private final PrefValue<Double> kV        = new PrefValue<>("shooter_kV",           0.00018, this);
  private final PrefValue<Double> kTol      = new PrefValue<>("shooter_PIDTolerance", 10.0,    this);
  private final PrefValue<Double> kDfilter  = new PrefValue<>("shooter_kDfilter",     0.0,     this);
  private final PrefValue<Double> kRampRate = new PrefValue<>("shooter_kRampRate",    0.0,     this);
  private final PrefValue<Double> feederSpeed = new PrefValue<>("feederSpeed",        0.3,     this);

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

    // Wire onChange callbacks so dashboard edits reconfigure the motor controller
    kP.onChange(x -> setSparkClosedLoopConfig());
    kI.onChange(x -> setSparkClosedLoopConfig());
    kD.onChange(x -> setSparkClosedLoopConfig());
    kV.onChange(x -> setSparkClosedLoopConfig());
    kTol.onChange(x -> setSparkClosedLoopConfig());
    kDfilter.onChange(x -> setSparkClosedLoopConfig());
    kRampRate.onChange(x -> setSparkClosedLoopConfig());

    // Apply gain values to the motor controller
    setSparkClosedLoopConfig();
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
    rpmSetpoint = MathUtil.clamp(rpm, 0.0, Constants.ShooterConstants.maxRPM);
    closedLoopController.setSetpoint(rpmSetpoint, ControlType.kVelocity, currentSlot);
    if(isOpenLoop()) {timeAtSpeed.restart();}
  }

  public void setRPMsetpoint(double rpm, ClosedLoopSlot slot) {
    rpmSetpoint = MathUtil.clamp(rpm, 0.0, Constants.ShooterConstants.maxRPM);
    closedLoopController.setSetpoint(rpmSetpoint, ControlType.kVelocity, slot);
    currentSlot = slot;
    if(isOpenLoop()) {timeAtSpeed.restart();}
  }


  /**
   * Apply the currently-stored gain values to the motor controller.
   */
  private void setSparkClosedLoopConfig() {
    SparkMaxConfig cfg = new SparkMaxConfig();

    // slot zero has PID and feedforward
    ClosedLoopConfig cl0 = new ClosedLoopConfig()
        .pid(kP.get(), kI.get(), kD.get(), ClosedLoopSlot.kSlot0)
        .dFilter(kDfilter.get(), ClosedLoopSlot.kSlot0)
        .apply(new FeedForwardConfig().kV(kV.get(), ClosedLoopSlot.kSlot0))
        .allowedClosedLoopError(kTol.get(), ClosedLoopSlot.kSlot0);
    cfg.apply(cl0);

    // slot one has only feedforward
    ClosedLoopConfig cl1 = new ClosedLoopConfig()
        .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot1)
        .apply(new FeedForwardConfig().kV(kV.get(), ClosedLoopSlot.kSlot1))
        .allowedClosedLoopError(kTol.get(), ClosedLoopSlot.kSlot1);
    cfg.apply(cl1);

    cfg.closedLoopRampRate(kRampRate.get());

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
      return Math.abs(currentRPM - rpmSetpoint) <= kTol.get();
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
    feederMotor.set(feederSpeed.get());
  }

  public void feedBackward() {
    feederMotor.set(-feederSpeed.get());
  }

  public void setFeederSpeed(double speed) {
    feederSpeed.set(speed);
  }

  public double getFeederSpeed() {
    return feederSpeed.get();
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


  public Command incrementSpeedCommand() {
    return new RunCommand(() -> {
      setRPMsetpoint(getRPMsetpoint() + Constants.ShooterConstants.manualRPMincrement);
    }, this);
  }

  public Command decrementSpeedCommand() {
    return new RunCommand(() -> {
      setRPMsetpoint(getRPMsetpoint() - Constants.ShooterConstants.manualRPMincrement);
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
    // PrefValue-backed tunable properties (auto-registered)
    PrefValue.addAllBuilderProperties(this, builder);

    // Non-preference properties
    builder.addDoubleProperty("Shooter RPM", () -> getRPM(), null);
    builder.addDoubleProperty("Shooter RPM raw", () -> currentRPM, null);
    builder.addDoubleProperty("Shooter SetpointRPM", () -> getRPMsetpoint(),
        (x) -> setRPMsetpoint(x));
    builder.addBooleanProperty("Ready?", () -> isReady(), null);
    builder.addBooleanProperty("Danger Mode", () -> isDangerMode(), (x) -> setDangerMode(x));
    builder.addBooleanProperty("Save Prefs", () -> false, (x) -> {
      if (x) PrefValue.saveObjectPrefs(this);
    });
    builder.addDoubleProperty("shoot motor output", () -> shooterMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("feed motor output", () -> feederMotor.get(), null);
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