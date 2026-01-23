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
    public RelativeEncoder encoder;
    public PIDController PID;
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double maxRPM = 2000.0;
    public double mSetpoint = 0.0;
    public double speed = 0.0;
    public double tolerance = Constants.ShooterConstants.PIDTolerance;

    public Shooter(int motorID){
    shooterMotor = new SparkMax(motorID, MotorType.kBrushless);
    shooterMotor.configure(new SparkMaxConfig().inverted(false)
                                                  .idleMode(IdleMode.kBrake),
                              ResetMode.kResetSafeParameters,
                              PersistMode.kNoPersistParameters
                             );
    encoder = shooterMotor.getEncoder();

    PID = new PIDController(kP, kI, kD);
    PID.setTolerance(tolerance);
    PID.reset();

    
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

    public Boolean atSetpoint(){
        return PID.atSetpoint();
    }

  

    @Override
    public void periodic() {
        double currentVelocity = encoder.getVelocity(); //rpm
        double output = PID.calculate(currentVelocity, mSetpoint);
        shooterMotor.set(output);
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
        builder.addDoubleProperty("MaxRPM", () -> getMaxRPM(),
                                        (x) -> setMaxRPM(x));
        builder.addDoubleProperty("Set RPM", () -> encoder.getVelocity(),
                                        (x) -> setRPM(x));
        builder.addDoubleProperty("Current RPM", () -> encoder.getVelocity(), null);
        

       builder.addBooleanProperty("At Setpoint?", () -> atSetpoint(), null);                                 
    }
    
}
