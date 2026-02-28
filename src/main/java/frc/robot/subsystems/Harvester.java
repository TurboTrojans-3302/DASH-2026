package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Harvester extends SubsystemBase{

    private SparkMax m_harvestMotor;
    private RelativeEncoder m_harvestEncoder;
    private DriveTrain m_driveTrain;

    public Harvester(DriveTrain drivetrain){
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

    public void setSpeed(double speed){
        m_harvestMotor.set(speed);
    }

    public void stop(){
        m_harvestMotor.set(0.0);
    }

}
