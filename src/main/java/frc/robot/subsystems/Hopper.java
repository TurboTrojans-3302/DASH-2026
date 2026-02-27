package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.MathUtil;
//TODO reimplement this whole thing with closed-loop position control.
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
        leftSparkConfig.apply(new SoftLimitConfig());

        SparkMaxConfig rightSparkConfig = new SparkMaxConfig().apply(leftSparkConfig);
        rightSparkConfig.inverted(true); // invert right motor so that positive speed extends both sides in the same
                                         // direction
        rightSparkConfig.encoder.inverted(true);

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
        // If we're at the max position and trying to extend, don't allow it
        if (atMaxPosition() && increment > 0) {
            stop();
            return;
        }

        // If we're at the min position and trying to retract, don't allow it
        if (atMinPosition() && increment < 0) {
            stop();
            return;
        }

        double motorSpeed = MathUtil.clamp(kV * Math.signum(increment), -1.0, 1.0); // simple feedforward based on direction of movement
        leftMotor.set(motorSpeed);
        rightMotor.set(motorSpeed);
    }

    /**
     * Stop both hopper motors.
     */
    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    @Override
    public void periodic() {
        if(leftHardLimit()) { leftEncoder.setPosition(0); } 
        if(rightHardLimit()) { rightEncoder.setPosition(0); }
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

    /**
     * Command that expands the hopper at the default speed while scheduled.
     */
    public Command expandCommand() {
        return new FunctionalCommand(() -> move(moveIncrement), // init: set speed to expand
                () -> {
                    if (atSoftMaxL())
                        leftMotor.set(0);
                    if (atSoftMaxR())
                        rightMotor.set(0);
                },
                (interrupted) -> stop(), // On end (whether interrupted or not), stop the motors
                () -> atMaxPosition(), // End condition: stop when hopper reaches max position
                this);
    }

    public Command retractCommand() {
        return new FunctionalCommand(() -> move(-moveIncrement), // init: set speed to retract
                () -> {
                    if (atSoftMinL())
                        leftMotor.set(0);
                    if (atSoftMinR())
                        rightMotor.set(0);
                },
                (interrupted) -> stop(), // On end (whether interrupted or not), stop the motors
                () -> atMinPosition(), // End condition: stop when hopper reaches min position
                this);
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop, this);
    }

    public void loadPreferences() {
        if (Preferences.containsKey(Constants.HopperConstants.moveIncrementKey)) {
            System.out.println("Loading Hopper  values from preferences");
            moveIncrement = Preferences.getDouble(Constants.HopperConstants.moveIncrementKey,
                    HopperConstants.moveIncrementDefault);
            softMax = Preferences.getDouble(Constants.HopperConstants.maxPositionKey,
                    HopperConstants.maxPositionDefault);
            softMin = Preferences.getDouble(Constants.HopperConstants.minPositionKey,
                    HopperConstants.minPositionDefault);
            kV = Preferences.getDouble(Constants.HopperConstants.kVkey, HopperConstants.kVdefault);
        } else {
            System.out.println("No hopper prefs found. Using default values");
        }
    }

    public void savePreferences() {
        System.out.println("Saving hopper values to preferences");
        Preferences.setDouble(Constants.HopperConstants.moveIncrementKey, moveIncrement);
        Preferences.setDouble(Constants.HopperConstants.maxPositionKey, softMax);
        Preferences.setDouble(Constants.HopperConstants.minPositionKey, softMin);
        Preferences.setDouble(Constants.HopperConstants.kVkey, kV);
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
        builder.addDoubleProperty("Left Motor Position", () -> leftEncoder.getPosition(), null);
        builder.addDoubleProperty("Right Motor Position", () -> rightEncoder.getPosition(), null);
        builder.addDoubleProperty("kV", () -> kV, (x) -> { kV = x; });
        builder.addBooleanProperty("Left Limit", () -> leftHardLimit(), null);
        builder.addBooleanProperty("Right Limit", () -> rightHardLimit(), null);
        builder.addBooleanProperty("Save Prefs", () -> false, (x) -> { if (x) savePreferences(); });
    }
}
