package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private SparkMax shooterMotor;
  private SparkMax feederMotor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController closedLoopController;
  private double rpmSetpoint = 0.0;
  private boolean PIDEnabled = false;
  private Timer timeAtSpeed = new Timer();
  private double kP;
  private double kI;
  private double kD;
  private double kV;
  private double kTol;

  private final double spinUpTime = 2.0; // seconds that the shooter must be at the target speed before we consider it
                                         // "ready" to shoot, can be tuned based on how long it takes for the shooter to
                                         // stabilize at the target speed after a change

  public Shooter(int shooterMotorID, int feederMotorID) {
    shooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushless);
    shooterMotor.configure(new SparkMaxConfig().inverted(false)
        .idleMode(IdleMode.kCoast),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    encoder = shooterMotor.getEncoder();

    // Initialize local gain values from defaults and configure controller
    kP = Constants.ShooterConstants.kPdefault;
    kI = Constants.ShooterConstants.kIdefault;
    kD = Constants.ShooterConstants.kDdefault;
    kV = Constants.ShooterConstants.kVdefault;
    kTol = Constants.ShooterConstants.PIDToleranceDefault;
    setPIDVT(kP, kI, kD, kV, kTol);
    closedLoopController = shooterMotor.getClosedLoopController();

    feederMotor = new SparkMax(feederMotorID, MotorType.kBrushless);
    feederMotor.configure(new SparkMaxConfig().inverted(false)
        .idleMode(IdleMode.kBrake),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    rpmSetpoint = 0.0;
    shooterMotor.set(0); // sets it to zero because it is the default
    feederMotor.set(0);
  }

  public void setRPMsetpoint(double rpm) {
    rpmSetpoint = MathUtil.clamp(rpm, 0.0, Constants.ShooterConstants.maxRPM);
    closedLoopController.setSetpoint(rpmSetpoint, SparkBase.ControlType.kVelocity,
        isPIDEnabled() ? ClosedLoopSlot.kSlot0 : ClosedLoopSlot.kSlot1);
  }

  /**
   * Configure the SparkMax closed-loop PID and feedforward values.
   * This centralizes configuration so sendable/profile updates and preference
   * loads
   * can reuse the same routine.
   */
  private void setPIDVT(double p, double i, double d, double v, double tol) {
    // store locally
    kP = p;
    kI = i;
    kD = d;
    kV = v;
    kTol = tol;

    // Build a single SparkMaxConfig and apply both slot configs into it
    SparkMaxConfig cfg = new SparkMaxConfig();

    ClosedLoopConfig cl0 = new ClosedLoopConfig()
        .pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
        .apply(new FeedForwardConfig().kV(kV, ClosedLoopSlot.kSlot0))
        .allowedClosedLoopError(kTol, ClosedLoopSlot.kSlot0);
    cfg.apply(cl0);

    ClosedLoopConfig cl1 = new ClosedLoopConfig()
        .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot1)
        .apply(new FeedForwardConfig().kV(kV, ClosedLoopSlot.kSlot1))
        .allowedClosedLoopError(kTol, ClosedLoopSlot.kSlot1);
    cfg.apply(cl1);

    shooterMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Convenience overload that reapplies the currently-stored kP/kI/kD/kV/kTol
   * values to the motor controller.
   */
  private void setPIDVT() {
    setPIDVT(kP, kI, kD, kV, kTol);
  }

  public double getRPM() {
    return encoder.getVelocity();
  }

  public double getMotorPctOutput() {
    return shooterMotor.get();
  }

  public Boolean ready() {
    if (isPIDEnabled()) {
      return closedLoopController.isAtSetpoint();
    } else {
      return timeAtSpeed.hasElapsed(spinUpTime);
    }
  }

  public void enablePID(Boolean enable) {
    if (enable ^ PIDEnabled) {
      // changing state: either enabling or disabling PID
      PIDEnabled = enable;
      setRPMsetpoint(getRPM());
      closedLoopController.setIAccum(0.0);
    }
  }

  public boolean isPIDEnabled() {
    return PIDEnabled;
  }

  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
  }

  public void stop() {
    setRPMsetpoint(0.0);
    setFeederSpeed(0.0);
  }

  @Override
  public void periodic() {
    // Closed-loop control is handled on the SparkMax when PIDEnabled is true.
    // For open-loop (PID disabled) we do not alter motor output here; callers
    // should
    // use setMotorPctOutput(...) to set direct percent output.
  }

  public void loadPreferences() {
    if (Preferences.containsKey(Constants.ShooterConstants.kPkey)) {
      System.out.println("Loading shooter PID values from preferences");
      kP = Preferences.getDouble(Constants.ShooterConstants.kPkey, Constants.ShooterConstants.kPdefault);
      kI = Preferences.getDouble(Constants.ShooterConstants.kIkey, Constants.ShooterConstants.kIdefault);
      kD = Preferences.getDouble(Constants.ShooterConstants.kDkey, Constants.ShooterConstants.kDdefault);
      kV = Preferences.getDouble(Constants.ShooterConstants.kVkey, Constants.ShooterConstants.kVdefault);
    // Apply loaded gains and tolerance to the motor controller
    double tol = Preferences.getDouble(Constants.ShooterConstants.PIDToleranceKey,
      Constants.ShooterConstants.PIDToleranceDefault);
    setPIDVT(kP, kI, kD, kV, tol);
    } else {
      System.out.println("No shooter prefs found. Using default values");
    }
  }

  public void savePreferences() {
    System.out.println("Saving shooter PID values to preferences");
    Preferences.setDouble(Constants.ShooterConstants.kPkey, kP);
    Preferences.setDouble(Constants.ShooterConstants.kIkey, kI);
    Preferences.setDouble(Constants.ShooterConstants.kDkey, kD);
    Preferences.setDouble(Constants.ShooterConstants.kVkey, kV);
    // No direct API to read closed-loop allowed error from the controller here;
    // save the default
    Preferences.setDouble(Constants.ShooterConstants.PIDToleranceKey, Constants.ShooterConstants.PIDToleranceDefault);
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

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Expose closed-loop gains and allow updating them by reconfiguring the
    // SparkMax
    builder.addDoubleProperty("kP", () -> kP,
        (x) -> { kP = x; setPIDVT(); });
    builder.addDoubleProperty("kI", () -> kI,
        (x) -> { kI = x; setPIDVT(); });
    builder.addDoubleProperty("kD", () -> kD,
        (x) -> { kD = x; setPIDVT(); });
    builder.addDoubleProperty("kV", () -> kV,
        (x) -> { kV = x; setPIDVT(); });
    builder.addDoubleProperty("kTolerance", () -> kTol,
        (x) -> { kTol = x; setPIDVT(); });
    builder.addDoubleProperty("Shooter RPM", () -> getRPM(), null);
    builder.addDoubleProperty("Shooter SetpointRPM", () -> getRPMsetpoint(),
        (x) -> setRPMsetpoint(x));
    builder.addBooleanProperty("Ready?", () -> ready(), null);
    builder.addBooleanProperty("PID Enabled", () -> PIDEnabled, null);

    builder.addDoubleProperty("Feeder Speed", () -> feederMotor.get(),
        (x) -> setFeederSpeed(x));

    builder.addBooleanProperty("Save Prefs", () -> false, (x) -> {
      if (x)
        savePreferences();
    });
    builder.addDoubleProperty("motor output", () -> shooterMotor.get(), null);
  }

  public double getRPMsetpoint() {
    return rpmSetpoint;
  }
}
