package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private PIDController PID;
  private SimpleMotorFeedforward feedforward;
  private boolean PIDEnabled = false;
  private double feederSpeed = Constants.ShooterConstants.feederSpeedDefault;
  private boolean dangerMode = false;
  private Timer timeAtSpeed = new Timer();
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

    PID = new PIDController(Constants.ShooterConstants.kPdefault,
        Constants.ShooterConstants.kIdefault,
        Constants.ShooterConstants.kDdefault);
    PID.setTolerance(Constants.ShooterConstants.PIDToleranceDefault);
    PID.reset();

    feedforward = new SimpleMotorFeedforward(0.0, Constants.ShooterConstants.kVdefault);

    loadPreferences();

    feederMotor = new SparkMax(feederMotorID, MotorType.kBrushless);
    feederMotor.configure(new SparkMaxConfig().inverted(false)
        .idleMode(IdleMode.kBrake),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    PID.setSetpoint(0.0);
    shooterMotor.set(0); // sets it to zero because it is the default
    feederMotor.set(0);

    rangeRPMtable.put(1.0, 2000.0);
    rangeRPMtable.put(2.0, 3000.0);

    dxSensor = new LaserCan(Constants.CanIds.DX_SENSOR_CAN_ID);
  }

  public void setRPMsetpoint(double rpm) {
    coasting = false;
    PID.setSetpoint(MathUtil.clamp(rpm, 0.0, Constants.ShooterConstants.maxRPM));

    if(!PIDEnabled){
      setMotorPctOutput(feedforward.calculate(PID.getSetpoint()));
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
      return PID.atSetpoint();
    } else {
      return timeAtSpeed.hasElapsed(spinUpTime);
    }
  }

  public void enablePID(boolean enable) {
    if (enable && !PIDEnabled) {
      PID.reset();
      setRPMsetpoint(getRPM());
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

  public double getDXsensor(){
    return dxSensor.getMeasurement().distance_mm;
  }

  private void coast() {
    coasting = true;
    setMotorPctOutput(0);
  }

  @Override
  public void periodic() {

    if (isPIDEnabled() && !coasting) {
      double currentVelocity = encoder.getVelocity(); // rpm
      double output = feedforward.calculate(PID.getSetpoint());
      output += PID.calculate(currentVelocity);
      setMotorPctOutput(output);
    }

  }

  public void loadPreferences() {
    if (Preferences.containsKey(Constants.ShooterConstants.kPkey)) {
      System.out.println("Loading shooter PID values from preferences");
      PID.setP(Preferences.getDouble(Constants.ShooterConstants.kPkey, Constants.ShooterConstants.kPdefault));
      PID.setI(Preferences.getDouble(Constants.ShooterConstants.kIkey, Constants.ShooterConstants.kIdefault));
      PID.setD(Preferences.getDouble(Constants.ShooterConstants.kDkey, Constants.ShooterConstants.kDdefault));
      feedforward.setKv(Preferences.getDouble(Constants.ShooterConstants.kVkey, Constants.ShooterConstants.kVdefault));
      PID.setTolerance(Preferences.getDouble(Constants.ShooterConstants.PIDToleranceKey,
          Constants.ShooterConstants.PIDToleranceDefault));
      feederSpeed = Preferences.getDouble(Constants.ShooterConstants.feederSpeedKey,
          Constants.ShooterConstants.feederSpeedDefault);
    } else {
      System.out.println("No shooter prefs found. Using default values");
    }
  }

  public void savePreferences() {
    System.out.println("Saving shooter PID values to preferences");
    Preferences.setDouble(Constants.ShooterConstants.kPkey, PID.getP());
    Preferences.setDouble(Constants.ShooterConstants.kIkey, PID.getI());
    Preferences.setDouble(Constants.ShooterConstants.kDkey, PID.getD());
    Preferences.setDouble(Constants.ShooterConstants.kVkey, feedforward.getKv());
    Preferences.setDouble(Constants.ShooterConstants.PIDToleranceKey, PID.getErrorTolerance());
    Preferences.setDouble(Constants.ShooterConstants.feederSpeedKey, feederSpeed);
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
    builder.addDoubleProperty("kP", () -> PID.getP(),
        (x) -> PID.setP(x));
    builder.addDoubleProperty("kI", () -> PID.getI(),
        (x) -> PID.setI(x));
    builder.addDoubleProperty("kD", () -> PID.getD(),
        (x) -> PID.setD(x));
    builder.addDoubleProperty("kV", () -> feedforward.getKv(),
        (x) -> feedforward.setKv(x));
    builder.addDoubleProperty("kTolerance", () -> PID.getErrorTolerance(),
        (x) -> PID.setTolerance(x));
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
    return PID.getSetpoint();
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

  public Double getRPMforRange(double range){
    return rangeRPMtable.get(range);
  }
}
