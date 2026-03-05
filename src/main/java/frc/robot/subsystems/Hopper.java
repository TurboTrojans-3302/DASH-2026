package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Hopper subsystem controls two Neo550 motors to expand or retract the hopper.
 */
public class Hopper extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private final DigitalInput leftContractedLimitSwitch = new DigitalInput(Constants.DigitalIO.kHopperLeftContractedLimitSwitchDio);
    private final DigitalInput rightContractedLimitSwitch = new DigitalInput(Constants.DigitalIO.kHopperRightContractedLimitSwitchDio);
    private double softMax = HopperConstants.maxPositionDefault;
    private double softMin = HopperConstants.minPositionDefault;
    private double kV = HopperConstants.kVdefault;
    private double kP = HopperConstants.kPdefault;
    private double kI = HopperConstants.kIdefault;
    private double kD = HopperConstants.kDdefault;
    private double posTolerance = HopperConstants.posToleranceDefault;
    private double maxVelocity     = HopperConstants.maxVelocityDefault;
    private double maxAcceleration = HopperConstants.maxAccelerationDefault;
    private boolean PIDEnabled = false;
    private double positionSetpoint = 0.0;

    private final ProfiledPIDController leftPID  = new ProfiledPIDController(
            HopperConstants.kPdefault, HopperConstants.kIdefault, HopperConstants.kDdefault,
            new TrapezoidProfile.Constraints(HopperConstants.maxVelocityDefault, HopperConstants.maxAccelerationDefault));
    private final ProfiledPIDController rightPID = new ProfiledPIDController(
            HopperConstants.kPdefault, HopperConstants.kIdefault, HopperConstants.kDdefault,
            new TrapezoidProfile.Constraints(HopperConstants.maxVelocityDefault, HopperConstants.maxAccelerationDefault));


    // default percent output to use for extend/retract if caller doesn't specify
    private double moveIncrement = HopperConstants.moveIncrementDefault;

    public Hopper(int leftMotorID, int rightMotorID) {
        loadPreferences();

        leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);

        configureSparkMaxes();

        leftMotor.set(0);
        leftEncoder = leftMotor.getEncoder();
        leftEncoder.setPosition(0); // reset encoder position on startup

        rightMotor.set(0);
        rightEncoder = rightMotor.getEncoder();
        rightEncoder.setPosition(0); // reset encoder position on startup

    }

    private void configureSparkMaxes() {
        SparkMaxConfig leftSparkConfig = new SparkMaxConfig();
        leftSparkConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

        SparkMaxConfig rightSparkConfig = new SparkMaxConfig().apply(leftSparkConfig);
        rightSparkConfig.inverted(true); // invert right motor 

        leftMotor.configure(leftSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(rightSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Convenience constructor that uses CAN IDs from
     * {@link frc.robot.Constants.HopperConstants}.
     */
    public Hopper() {
        this(Constants.CanIds.kHopperLeftMotorCanId, Constants.CanIds.kHopperRightMotorCanId);
    }

    public boolean isEmpty() {
        // Placeholder for actual sensor check to determine if hopper is empty
        // This could be implemented with a limit switch, ultrasonic sensor, etc.
        return false; // Assume not empty for now
    }

    public void move(double increment) {
        if (PIDEnabled) {
            // PID runs in periodic(); nudge the setpoint by increment
            setPosition(positionSetpoint + increment);
        } else {
            double motorSpeed = MathUtil.clamp(kV * Math.signum(increment), -1.0, 1.0);
            setMotorPctOutput(motorSpeed, motorSpeed);
        }
    }

    public void setPosition(double pos) {
        if(PIDEnabled) {
            positionSetpoint = MathUtil.clamp(pos, softMin, softMax);
            leftPID.setGoal(positionSetpoint);
            rightPID.setGoal(positionSetpoint);
        }else{
            DriverStation.reportWarning("Can't set hopper position while PID is disabled!", false);
        }
    }

    public boolean isPIDEnabled() {
        return PIDEnabled;
    }

    public void setPIDEnabled(boolean enabled) {
        if (enabled && !PIDEnabled) {
            PIDEnabled = enabled;
            // Snap setpoint to current average position and reset controllers
            double currentPos = (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
            setPosition(currentPos);
            leftPID.reset(currentPos);
            rightPID.reset(currentPos);
        }
        PIDEnabled = enabled;
    }

    /**
     * Stop both hopper motors.
     */
    public void stop() {
        setMotorPctOutput(0, 0);
    }

    private void setMotorPctOutput(double leftSpeed, double rightSpeed) {
        leftSpeed  = MathUtil.clamp(leftSpeed,  -1.0, 1.0);
        rightSpeed = MathUtil.clamp(rightSpeed, -1.0, 1.0);
        if(atSoftMaxL()) leftSpeed = Math.min(0, leftSpeed); // if at max, only allow retracting (negative speed)
        if(atSoftMaxR()) rightSpeed = Math.min(0, rightSpeed); // if at max, only allow retracting (negative speed)
        if(atSoftMinL()) leftSpeed = Math.max(0, leftSpeed); // if at min, only allow extending (positive speed)
        if(atSoftMinR()) rightSpeed = Math.max(0, rightSpeed); // if at min, only allow extending (positive speed)

        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
    }

    @Override
    public void periodic() {
        if (leftHardLimit())  { leftEncoder.setPosition(0); }
        if (rightHardLimit()) { rightEncoder.setPosition(0); }

        if (PIDEnabled) {
            double leftOutput  = leftPID.calculate(leftEncoder.getPosition());
            double rightOutput = rightPID.calculate(rightEncoder.getPosition());
            setMotorPctOutput(leftOutput, rightOutput);
        }
    }

    public boolean atSoftMaxL() {
        return leftEncoder.getPosition() >= softMax;
    }

    public boolean atSoftMaxR() {
        return rightEncoder.getPosition() >= softMax;
    }

    public boolean atMaxPosition() {
        return atSoftMaxL() && atSoftMaxR();
    }

    public boolean atSoftMinL() {
        return leftEncoder.getPosition() <= softMin;
    }

    public boolean atSoftMinR() {
        return rightEncoder.getPosition() <= softMin;
    }

    public boolean atMinPosition() {
        return atSoftMinL() && atSoftMinR();
    }

    public Command stopCommand() {
        Command cmd = new InstantCommand(this::stop, this);
        cmd.setName("stopCommand");
        return cmd;
    }

    public Command manualMoveCommand(DoubleSupplier increment) {
        return new FunctionalCommand(() -> {},
                                     () -> move(increment.getAsDouble()), 
                                     (interrupted) -> stop(), 
                                     () -> false, 
                                     this);
    }

    public Command manualExpandCommand() {
        return manualMoveCommand(() -> moveIncrement);
    }

    public Command manualRetractCommand() {
        return manualMoveCommand(() -> -moveIncrement);
    }

    public Command setPositionCommand(DoubleSupplier targetPosition) {
        FunctionalCommand cmd = new FunctionalCommand(
            () -> { setPosition(targetPosition.getAsDouble()); },
            () -> {}, // periodic() handles the output
            (interrupted) -> { stop(); },
            () -> (leftPID.atGoal() && rightPID.atGoal()) || !PIDEnabled, // end command when at position or if PID is disabled
            this);
        cmd.setName("setPositionCommand");
        return cmd;
    }

    public Command expandCommand() {
        Command cmd = setPositionCommand(() -> softMax);
        cmd.setName("expandCommand");
        return cmd;
    }

    public Command retractCommand() {
        Command cmd = setPositionCommand(() -> softMin);
        cmd.setName("retractCommand");
        return cmd;
    }

    /** Apply current kP/kI/kD/tolerance/constraints to both PID controllers. */
    private void applyPIDGains() {
        leftPID.setPID(kP, kI, kD);
        rightPID.setPID(kP, kI, kD);
        leftPID.setTolerance(posTolerance);
        rightPID.setTolerance(posTolerance);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        leftPID.setConstraints(constraints);
        rightPID.setConstraints(constraints);
    }

    private void loadPreferences() {
        if (Preferences.containsKey(HopperConstants.moveIncrementKey)) {
            System.out.println("Loading Hopper  values from preferences");
            moveIncrement = Preferences.getDouble(HopperConstants.moveIncrementKey,
                    HopperConstants.moveIncrementDefault);
            softMax = Preferences.getDouble(HopperConstants.maxPositionKey,
                    HopperConstants.maxPositionDefault);
            softMin = Preferences.getDouble(HopperConstants.minPositionKey,
                    HopperConstants.minPositionDefault);
            kV = Preferences.getDouble(HopperConstants.kVkey, HopperConstants.kVdefault);
            kP = Preferences.getDouble(HopperConstants.kPkey, HopperConstants.kPdefault);
            kI = Preferences.getDouble(HopperConstants.kIkey, HopperConstants.kIdefault);
            kD = Preferences.getDouble(HopperConstants.kDkey, HopperConstants.kDdefault);
            posTolerance = Preferences.getDouble(HopperConstants.posToleranceKey, HopperConstants.posToleranceDefault);
            maxVelocity     = Preferences.getDouble(HopperConstants.maxVelocityKey,     HopperConstants.maxVelocityDefault);
            maxAcceleration = Preferences.getDouble(HopperConstants.maxAccelerationKey, HopperConstants.maxAccelerationDefault);
            applyPIDGains();
        } else {
            System.out.println("No hopper prefs found. Using default values");
        }
    }

    private void savePreferences() {
        System.out.println("Saving hopper values to preferences");
        Preferences.setDouble(HopperConstants.moveIncrementKey, moveIncrement);
        Preferences.setDouble(HopperConstants.maxPositionKey, softMax);
        Preferences.setDouble(HopperConstants.minPositionKey, softMin);
        Preferences.setDouble(HopperConstants.kVkey, kV);
        Preferences.setDouble(HopperConstants.kPkey, kP);
        Preferences.setDouble(HopperConstants.kIkey, kI);
        Preferences.setDouble(HopperConstants.kDkey, kD);
        Preferences.setDouble(HopperConstants.posToleranceKey, posTolerance);
        Preferences.setDouble(HopperConstants.maxVelocityKey,     maxVelocity);
        Preferences.setDouble(HopperConstants.maxAccelerationKey, maxAcceleration);
    }

    public boolean leftHardLimit() {
        return leftContractedLimitSwitch.get();
    }

    public boolean rightHardLimit() {
        return rightContractedLimitSwitch.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Expand Speed", () -> moveIncrement, (x) -> { moveIncrement = x; });
        builder.addDoubleProperty("Max Position", () -> softMax, (x) -> { softMax = x; });
        builder.addDoubleProperty("Min Position", () -> softMin, (x) -> { softMin = x; });
        builder.addDoubleProperty("Left Position", () -> leftEncoder.getPosition(), null);
        builder.addDoubleProperty("Right Position", () -> rightEncoder.getPosition(), null);
        builder.addDoubleProperty("Position Setpoint", () -> positionSetpoint, (x) -> setPosition(x));
        builder.addDoubleProperty("L Motor Out", () -> leftMotor.get(), null);
        builder.addDoubleProperty("R Motor Out", () -> rightMotor.get(), null);
        builder.addDoubleProperty("kV", () -> kV, (x) -> { kV = x; });
        builder.addDoubleProperty("kP", () -> kP, (x) -> { kP = x; applyPIDGains(); });
        builder.addDoubleProperty("kI", () -> kI, (x) -> { kI = x; applyPIDGains(); });
        builder.addDoubleProperty("kD", () -> kD, (x) -> { kD = x; applyPIDGains(); });
        builder.addDoubleProperty("Max Velocity",     () -> maxVelocity,     (x) -> { maxVelocity     = x; applyPIDGains(); });
        builder.addDoubleProperty("Max Acceleration", () -> maxAcceleration, (x) -> { maxAcceleration = x; applyPIDGains(); });
        builder.addBooleanProperty("PID Enabled", () -> isPIDEnabled(), (x) -> setPIDEnabled(x));
        builder.addDoubleProperty("PID Setpoint", () -> positionSetpoint, (x) -> setPosition(x));
        builder.addBooleanProperty("Left Limit", () -> leftHardLimit(), null);
        builder.addBooleanProperty("Right Limit", () -> rightHardLimit(), null);
        builder.addBooleanProperty("Save Prefs", () -> false, (x) -> { if (x) savePreferences(); });
    }
}
