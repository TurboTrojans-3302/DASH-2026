package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Harvester extends SubsystemBase{

    private SparkMax m_harvestMotor;
    private RelativeEncoder m_harvestEncoder;
    private DriveTrain m_driveTrain

    public Harvester(DriveTrain drivetrain){
        SparkMaxConfig config = new SparkMaxConfig().inverted(false)
                                                    .idleMode(IdleMode.kBreak)
        m_harvestMotor = new SparkMax(Constants.CanIds.kHarvesterMotorCanId, MotorType.kBrushless);
        m_harvestMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
        m_harvestEncoder = m_harvestMotor.getEncoder();
    }


    private double getForwardVelocity(){
        ChassisSpeeds speeds = m_driveTrain.getRobotVelocity();
        return Math.max(speeds.vxmetersPerSecond, 0.0); //return zero if we're going backwards
    }

    public void setSpeed(double speed){
        m_harvestMotor.set(speed)
    }

    public void stop(){
        m_harvestMotor.set(0.0);
    }

}
