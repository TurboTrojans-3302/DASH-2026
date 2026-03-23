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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HarvesterConstants;
import frc.utils.PrefValue;

public class Harvester extends SubsystemBase {

  private SparkMax m_harvestMotor;
  private RelativeEncoder m_harvestEncoder;

  private final PrefValue<Double> kV = new PrefValue<>("harvester_kV", HarvesterConstants.kVdefault, this);
  private final PrefValue<Double> kP = new PrefValue<>("harvester_kP", HarvesterConstants.kPdefault, this);
  private final PrefValue<Double> pullInRPM = new PrefValue<>("harvesterPullInRPM", 200.0, this);

  private PIDController PID = new PIDController(kP.get(), 0, 0);
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, kV.get());
  private boolean PIDEnabled = false;

  private double setpoint = 0.0;

  public Harvester() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.apply(SparkMaxConfig.Presets.REV_NEO);
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.apply(new EncoderConfig().velocityConversionFactor(1 / 16.0)); // 16:1 gear reduction

    m_harvestMotor = new SparkMax(Constants.CanIds.kHarvesterMotorCanId, MotorType.kBrushless);
    m_harvestMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_harvestEncoder = m_harvestMotor.getEncoder();

    PID.setTolerance(HarvesterConstants.kTolDefault);
    kV.onChange((x) -> feedforward.setKv(x));
    kP.onChange((x) -> PID.setP(x));
  }

  @Override
  public void periodic() {
    if (setpoint == 0.0) {
      m_harvestMotor.setVoltage(0.0);
      return;
    }

    double feedforwardOutput = feedforward.calculate(setpoint);

    if (isPIDEnabled()) {
      double PIDoutput = PID.calculate(getRPM(), setpoint);
      m_harvestMotor.setVoltage(feedforwardOutput + PIDoutput);
    } else {
      m_harvestMotor.setVoltage(feedforwardOutput);
    }
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

  public boolean isOn(){
    return Math.abs(getRPM()) > 0.01;
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
          setSpeed(pullInRPM.get()); //shouldn't this be a doublesupplier?
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
          setSpeed(-pullInRPM.get());
        },
        (x) -> {
          stop();
        },
        () -> false,
        this);
  }

  public void adjustPullInRPM(double delta) {
    if(setpoint == pullInRPM.get() || setpoint == -pullInRPM.get()){
      setSpeed(Math.signum(setpoint) * (pullInRPM.get() + delta));
    }
    pullInRPM.set(pullInRPM.get() + delta);
  }

  public Command StopCommand() {
    return new InstantCommand(() -> setSpeed(0.0), this);
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    PrefValue.addAllBuilderProperties(this, builder);
    builder.addDoubleProperty("Harvester RPM", () -> getRPM(), null);
    builder.addDoubleProperty("motor output", () -> m_harvestMotor.get(), null);
    builder.addBooleanProperty("Save Prefs", () -> false, (x) -> { if (x) PrefValue.saveObjectPrefs(this); });
  }
}
