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

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Harvester extends SubsystemBase{

    private SparkMax m_harvestMotor;
    private RelativeEncoder m_harvestEncoder;
    private DriveTrain m_driveTrain;
    private Hopper m_hopper;
    private double speedConversionConstant = Constants.HarvesterConstants.speedConversionConstantDefault;
    private double pullInRPM = Constants.HarvesterConstants.pullInRPMDefault;
     
    public Harvester(DriveTrain drivetrain, Hopper hopper){
        m_driveTrain = drivetrain;
        m_hopper = hopper;
        SparkMaxConfig config = new SparkMaxConfig();
        config.apply(SparkMaxConfig.Presets.REV_NEO);
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.apply(new EncoderConfig().velocityConversionFactor(1/16.0));

        m_harvestMotor = new SparkMax(Constants.CanIds.kHarvesterMotorCanId, MotorType.kBrushless);
        m_harvestMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        m_harvestEncoder = m_harvestMotor.getEncoder();
        
    }


    private double getForwardVelocity(){
        ChassisSpeeds speeds = m_driveTrain.getRobotVelocity();
        return Math.max(speeds.vxMetersPerSecond, 0.0); //return zero if we're going backwards
    }

    public void setSpeed(double RPM){
        double speed = RPM * speedConversionConstant;
        m_harvestMotor.set(speed);
    }

    public double getRPM(){
        return m_harvestEncoder.getVelocity();
    }
    
    public void stop(){
        m_harvestMotor.set(0.0);
    }

    public Command SetSpeedCommand(DoubleSupplier RPM){
        return new InstantCommand(()-> pullInRPM = RPM.getAsDouble(), this);
    }

    public Command PullInCommand(){
        return new FunctionalCommand(
          ()->{},
          ()->{setSpeed(pullInRPM);},
          (x)->{stop();},
          ()->false,
          this);
    }

    public Command PushOutCommand(){
        return new FunctionalCommand(
          ()->{},
          ()->{setSpeed(-pullInRPM);},
          (x)->{stop();},
          ()->false,
          this);
    }

    public Command StopCommand(){
        return new InstantCommand(()-> setSpeed(0.0), this);
    }



    public void loadPreferences() {
    if (Preferences.containsKey(Constants.HarvesterConstants.speedConversionConstantKeyDefault)) {
      System.out.println("Loading harvester values from preferences");
      speedConversionConstant = Preferences.getDouble(Constants.HarvesterConstants.speedConversionConstantKeyDefault,
          Constants.HarvesterConstants.speedConversionConstantDefault);
    } else {
      System.out.println("No harvester prefs found. Using default values");
    }
  }

  public void savePreferences() {
    System.out.println("Saving Harvester values to preferences");
    Preferences.setDouble(Constants.HarvesterConstants.speedConversionConstantKeyDefault, speedConversionConstant);
  }

  @Override
  public void periodic(){
    if (m_hopper.atMinPosition()){
      stop();  
    }

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Speed Conversion Constant", () -> speedConversionConstant, (x) -> { speedConversionConstant = x; });
    builder.addDoubleProperty("Pull In RPM", () -> pullInRPM, (x) -> { pullInRPM = x; });
    builder.addDoubleProperty("Harvester RPM", () -> getRPM(), null);
    builder.addBooleanProperty("Save Prefs", () -> false, (x) -> { if (x) savePreferences(); });
    builder.addDoubleProperty("motor output", () -> m_harvestMotor.get(), null);
  }
}
