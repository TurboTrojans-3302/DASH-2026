package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    public SparkMax shooterMotor;
    public SparkMax feederMotor;
    public RelativeEncoder encoder;
    public PIDController PID;
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double maxRPM = 2000.0;
    public double mSetpoint = 0.0;
    public double shooterSpeed = 0.0;
    public double feederSpeed = 0.0;
    public double tolerance = Constants.ShooterConstants.PIDTolerance;
    public boolean PIDEnabled = true;
    public boolean feederEnabled = true;

    public Shooter(int shooterMotorID,int feederMotorID){
    shooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushless);
    shooterMotor.configure(new SparkMaxConfig().inverted(false)
                                                  .idleMode(IdleMode.kBrake),
                              ResetMode.kResetSafeParameters,
                              PersistMode.kNoPersistParameters
                             );
    feederMotor = new SparkMax(feederMotorID, MotorType.kBrushless);
    feederMotor.configure(new SparkMaxConfig().inverted(false)
                                                  .idleMode(IdleMode.kBrake),
                              ResetMode.kResetSafeParameters,
                              PersistMode.kNoPersistParameters
                             );
    encoder = shooterMotor.getEncoder();

    PID = new PIDController(kP, kI, kD);
    PID.setTolerance(tolerance);
    PID.reset();

    shooterMotor.set(shooterSpeed); //sets it to zero because it is the default
    feederMotor.set(feederSpeed);
    
    }

    public void setRPM(double speed){
       mSetpoint = MathUtil.clamp(speed, 0.0, maxRPM);
    }

    public void setMaxRPM(double RPM){
       maxRPM = RPM;
    }

    public double getMaxRPM(){
        return maxRPM;
    }

    public double getRPM(){
        return encoder.getVelocity();
    }

    public Boolean atSetpoint(){
        return PID.atSetpoint();
    }

    public void setPID(Boolean EnablePID){
        PIDEnabled = EnablePID;
    }

    public void toggleFeeder(Boolean activated){
        feederEnabled = activated;
    }

    public void setFeederSpeed(double speed){
        feederMotor.set(speed);
    }

  

    @Override
    public void periodic() {
        double currentVelocity = encoder.getVelocity(); //rpm
        if (PIDEnabled){
        double output = PID.calculate(currentVelocity, mSetpoint);
        shooterMotor.set(output);
    } 
    

    }

    @Override
    public void initSendable(SendableBuilder builder){
        super.initSendable(builder);
        builder.addDoubleProperty("kP", () -> PID.getP(),
                                        (x) -> PID.setP(x));
        builder.addDoubleProperty("kI", () -> PID.getI(),
                                        (x) -> PID.setI(x));
        builder.addDoubleProperty("kD", () -> PID.getD(),
                                        (x) -> PID.setD(x));
        builder.addDoubleProperty("kTolerance", () -> PID.getErrorTolerance(),
                                        (x) -> PID.setTolerance(x));
        builder.addDoubleProperty("MaxRPM Shooter", () -> getMaxRPM(),
                                        (x) -> setMaxRPM(x));
        builder.addDoubleProperty("Set Shooter RPM", () -> encoder.getVelocity(),
                                        (x) -> setRPM(x));
        
        builder.addDoubleProperty("Feeder Speed", () -> feederMotor.get(), 
                                    (x) -> setFeederSpeed(x));

        builder.addDoubleProperty("Current Shooter RPM", () -> getRPM(), null);
        builder.addBooleanProperty("At Setpoint?", () -> atSetpoint(), null);                                 
    }
    
}
