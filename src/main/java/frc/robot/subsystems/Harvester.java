package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
        config.inverted(false);
        config.idleMode(IdleMode.kBrake);

        m_harvestMotor = new SparkMax(Constants.CanIds.kHarvesterMotorCanId, MotorType.kBrushless);
        m_harvestMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
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
        return new InstantCommand(()-> setSpeed(pullInRPM), this);
    }

    public Command PushOutCommand(){
        return new InstantCommand(()-> setSpeed(-pullInRPM), this);
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
}
