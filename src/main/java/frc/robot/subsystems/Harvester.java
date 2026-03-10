package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Harvester extends SubsystemBase {

  private SparkMax m_harvestMotor;
  private RelativeEncoder m_harvestEncoder;
  private double pullInRPM = Constants.HarvesterConstants.pullInRPMDefault;

  private PIDController PID;
  private SimpleMotorFeedforward feedforward;
  private boolean PIDEnabled = false;

  private double setpoint = 0.0;

  public Harvester() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.apply(SparkMaxConfig.Presets.REV_NEO);
    config.inverted(true);
    config.idleMode(IdleMode.kBrake);
    config.apply(new EncoderConfig().velocityConversionFactor(1 / 16.0)); // 16:1 gear reduction

    m_harvestMotor = new SparkMax(Constants.CanIds.kHarvesterMotorCanId, MotorType.kBrushless);
    m_harvestMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_harvestEncoder = m_harvestMotor.getEncoder();
  }

  @Override
  public void periodic() {
    double feedforwardOutput = feedforward.calculate(setpoint);

    if (isPIDenabled()) {
      double PIDoutput = PID.calculate(getRPM(), setpoint);
      m_harvestMotor.setVoltage(feedforwardOutput + PIDoutput);
    } else {
      m_harvestMotor.setVoltage(feedforwardOutput);
    }
  }
    
  private boolean isPIDenabled() {
    return PIDEnabled;
  }
  
  public void enablePID(boolean enable) {
    if (enable && !PIDEnabled) { // turn PID on
      PID.reset();
      setSpeed(getRPM());
    }
    PIDEnabled = enable;
  }

  public boolean isPIDEnabled() {
    return PIDEnabled;
  }


  public double getRPM() {
    return m_harvestEncoder.getVelocity();
  }

  public void stop() {
    setSpeed(0.0);
  }

  public void setSpeed(double RPM) {
    setpoint = RPM;
  }

  public Command SetSpeedCommand(DoubleSupplier RPM) {
    return new InstantCommand(() -> setSpeed(RPM.getAsDouble()), this);
  }

  public Command PullInCommand() {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          setSpeed(pullInRPM);
        },
        (x) -> {
          stop();
        },
        () -> false,
        this);
  }

  public Command PushOutCommand() {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          setSpeed(-pullInRPM);
        },
        (x) -> {
          stop();
        },
        () -> false,
        this);
  }

  public Command StopCommand() {
    return new InstantCommand(() -> setSpeed(0.0), this);
  }

  public void loadPreferences() {
    if (Preferences.containsKey(Constants.HarvesterConstants.kVkey)) {
      System.out.println("Loading harvester values from preferences");
      feedforward.setKv(Preferences.getDouble(Constants.HarvesterConstants.kVkey,
          Constants.HarvesterConstants.kVdefault));
      PID.setP(Preferences.getDouble(Constants.HarvesterConstants.kPkey,
          Constants.HarvesterConstants.kPdefault));
      PID.setTolerance(Preferences.getDouble(Constants.HarvesterConstants.kTolKey,
          Constants.HarvesterConstants.kTolDefault));
    } else {
      System.out.println("No harvester prefs found. Using default values");
    }
  }

  public void savePreferences() {
    System.out.println("Saving Harvester values to preferences");
    Preferences.setDouble(Constants.HarvesterConstants.kVkey, feedforward.getKv());
    Preferences.setDouble(Constants.HarvesterConstants.kPkey, PID.getP());
    Preferences.setDouble(Constants.HarvesterConstants.kTolKey, PID.getErrorTolerance());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("kV", () -> feedforward.getKv(), (x) -> {
      feedforward.setKv(x);
    });
    builder.addDoubleProperty("kP", () -> PID.getP(), (x) -> {
      PID.setP(x);
    });
    builder.addDoubleProperty("kTol", () -> PID.getErrorTolerance(), (x) -> {
      PID.setTolerance(x);
    });
    builder.addDoubleProperty("Pull In RPM", () -> pullInRPM, (x) -> {
      pullInRPM = x;
    });
    builder.addDoubleProperty("Harvester RPM", () -> getRPM(), null);
    builder.addBooleanProperty("Save Prefs", () -> false, (x) -> {
      if (x)
        savePreferences();
    });
    builder.addDoubleProperty("motor output", () -> m_harvestMotor.getAppliedOutput(), null);
  }
}
