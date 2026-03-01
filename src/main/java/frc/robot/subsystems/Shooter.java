package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private double feederSpeed = Constants.ShooterConstants.feederSpeedDefault;
  private boolean dangerMode = false;
  private Timer timeAtSpeed = new Timer();
  private double kP;
  private double kI;
  private double kD;
  private double kV;
  private double kTol;
  private boolean coasting = false;
  private LaserCan dxSensor;


  private final InterpolatingDoubleTreeMap rangeRPMtable = new InterpolatingDoubleTreeMap();

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

    loadPreferences();

    feederMotor = new SparkMax(feederMotorID, MotorType.kBrushed);
    feederMotor.configure(new SparkMaxConfig().inverted(true)
        .idleMode(IdleMode.kBrake),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    rpmSetpoint = 0.0;
    shooterMotor.set(0); // sets it to zero because it is the default
    feederMotor.set(0);

    rangeRPMtable.put(1.0, 2000.0);
    rangeRPMtable.put(2.0, 3000.0);

    dxSensor = new LaserCan(Constants.CanIds.DX_SENSOR_CAN_ID);
  }

  public void setRPMsetpoint(double rpm) {
    coasting = false;
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

    if(!PIDEnabled){
      setMotorPctOutput(feedforward.calculate(rpmSetpoint));
    }
  }

  public double getRPM() {
    return encoder.getVelocity();
  }

  private void setMotorPctOutput(double speed) {
    shooterMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
    timeAtSpeed.restart();
  }

  public double getMotorPctOutput() {
    return shooterMotor.get();
  }

  public boolean ready() {
    if(isDangerMode()){
      return true;
    }
    if (isPIDEnabled()) {
      return closedLoopController.isAtSetpoint();
    } else {
      return timeAtSpeed.hasElapsed(spinUpTime);
    }
  }

  public void enablePID(boolean enable) {
    if (enable && !PIDEnabled) {
      setRPMsetpoint(getRPM());
      closedLoopController.setIAccum(0.0);
    }
    PIDEnabled = enable;
  }

  public boolean isPIDEnabled() {
    return PIDEnabled;
  }

  public boolean isDangerMode() {
    return dangerMode;
  }

  public void setDangerMode(boolean enabled) {
    dangerMode = enabled;
  }

  public void startFeeder() {
    feederMotor.set(feederSpeed);
  }

  public void startFeeder(double speed) {
    feederSpeed = speed;
    startFeeder();
  }

  public void stopFeeder() {
    feederMotor.set(0.0);
  } 

  public void stop() {
    coast();
    stopFeeder();
  }

  //TODO filter this value
  public double getDXsensor(){
    return dxSensor.getMeasurement().distance_mm;
  }

  private void coast() {
    coasting = true;
    setMotorPctOutput(0);
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
      
      feederSpeed = Preferences.getDouble(Constants.ShooterConstants.feederSpeedKey,
          Constants.ShooterConstants.feederSpeedDefault);
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
    Preferences.setDouble(Constants.ShooterConstants.PIDToleranceKey, kTol);
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
    builder.addBooleanProperty("PID Enabled", () -> PIDEnabled, null); // this is read-only on the dashboard.
    builder.addDoubleProperty("Feeder Speed", () -> feederSpeed,
        (x) -> feederSpeed = x);
    builder.addBooleanProperty("Danger Mode", () -> isDangerMode(), (x) -> setDangerMode(x));
    builder.addBooleanProperty("Save Prefs", () -> false, (x) -> {
      if (x)
        savePreferences();
    });
    builder.addDoubleProperty("shoot motor output", () -> shooterMotor.get(), null);
    builder.addDoubleProperty ("feed motor output",()->feederMotor.get(), null);
    builder.addDoubleProperty("dx sensor", ()->getDXsensor(), null);
  }

  public double getRPMsetpoint() {
    return rpmSetpoint;
  }

  public Command spinUpCommand(DoubleSupplier rpmSupplier) {
    return new RunCommand(() -> {
      setRPMsetpoint(rpmSupplier.getAsDouble());
    }, this);
  }

  public Command shootCommand(){
    return new FunctionalCommand(
      ()->{},
      ()->{ if(ready()){
              startFeeder();
            }else{
              stopFeeder();
            }},
      (interrupted)->{stopFeeder();},
      ()->false,
      this);
  }

  public Command setRangeCommand(DoubleSupplier rangeSupplier){
    return new InstantCommand(() -> {
      double range = rangeSupplier.getAsDouble();
      double targetRPM = getRPMforRange(range);
      setRPMsetpoint(targetRPM);
    }, this);
  }

  public Command reverseFeedCommand(){
    return new FunctionalCommand(
      ()->{},
      ()->{startFeeder(-feederSpeed);},
      (interrupted)->{stopFeeder();},
      ()->false,
      this);
  }

  public Double getRPMforRange(double range){
    return rangeRPMtable.get(range);
  }

}
