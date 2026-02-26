package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
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
  private Timer timeAtSpeed = new Timer();

  private final double spinUpTime = 1.0; // seconds that the shooter must be at the target speed before we consider it "ready" to shoot, can be tuned based on how long it takes for the shooter to stabilize at the target speed after a change

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

    feederMotor = new SparkMax(feederMotorID, MotorType.kBrushless);
    feederMotor.configure(new SparkMaxConfig().inverted(false)
        .idleMode(IdleMode.kBrake),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    PID.setSetpoint(0.0);    
    shooterMotor.set(0); // sets it to zero because it is the default
    feederMotor.set(0);
  }

  public void setRPMsetpoint(double rpm) {
    PID.setSetpoint(MathUtil.clamp(rpm, 0.0, Constants.ShooterConstants.maxRPM));
  }

  public double getRPM() {
    return encoder.getVelocity();
  }

  public void setMotorPctOutput(double speed) {
    shooterMotor.set(MathUtil.clamp(speed, 0.0, 1.0)); 
    timeAtSpeed.restart();
  } 

  public double getMotorPctOutput() {
    return shooterMotor.get();
  }

  public Boolean ready() {
    if(PIDEnabled){
      return PID.atSetpoint();
    } else {
      return timeAtSpeed.hasElapsed(spinUpTime); 
    }
  }

  public void enablePID(Boolean enable) {
    if(enable && !PIDEnabled) {
      PID.reset();
      setRPMsetpoint(getRPM());
    }
    PIDEnabled = enable;
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
    double currentVelocity = encoder.getVelocity(); // rpm
    if (PIDEnabled) {
      double output = PID.calculate(currentVelocity) + feedforward.calculate(PID.getSetpoint());
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
  }

  public Command incrementSpeedCommand() {
    return new RunCommand(() -> {
      if(PIDEnabled){
          setRPMsetpoint(getRPMsetpoint() + Constants.ShooterConstants.manualRPMincrement);
      } else {
          setMotorPctOutput(getMotorPctOutput() + Constants.ShooterConstants.manualPCTincrement);
      }
    }, this);
  }

  public Command decrementSpeedCommand() {
    return new RunCommand(() -> {
      if(PIDEnabled){
          setRPMsetpoint(getRPM() - Constants.ShooterConstants.manualRPMincrement);
      } else {
          setMotorPctOutput(getMotorPctOutput() - Constants.ShooterConstants.manualPCTincrement);
      }
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
    builder.addDoubleProperty("Shooter SetpointRPM", () -> PID.getSetpoint(),
        (x) -> PID.setSetpoint(x));
    builder.addBooleanProperty("Ready?", () -> ready(), null);
    builder.addBooleanProperty("PID Enabled", () -> PIDEnabled, null);

    builder.addDoubleProperty("Feeder Speed", () -> feederMotor.get(),
        (x) -> setFeederSpeed(x));

    builder.addBooleanProperty("Save Prefs", ()->false, (x)->{if(x) savePreferences();});
    builder.addDoubleProperty("motor output", ()->shooterMotor.get(), null);
  }

    public double getRPMsetpoint() {
      return PID.getSetpoint();
    }
}
