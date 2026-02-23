package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.MathUtil;

/**
 * Hopper subsystem controls two Neo550 motors to expand or retract the hopper.
 */
public class Hopper extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private double maxPosition = HopperConstants.maxPositionDefault;
    private double minPosition = HopperConstants.minPositionDefault;


    // default percent output to use for extend/retract if caller doesn't specify
    private double expandSpeed = HopperConstants.expandSpeedDefault;

    public Hopper(int leftMotorID, int rightMotorID) {
        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        // Basic configuration: brake when idle for holding position
        sparkConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

        leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
        leftMotor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        leftMotor.set(0);
        leftEncoder = leftMotor.getEncoder();

        rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);
        // right motor inverted to mirror left (common for two-sided mechanisms)
        rightMotor.configure(new SparkMaxConfig().inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(20),
                ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.set(0);
        rightEncoder = rightMotor.getEncoder();

        loadPreferences();
    }

    /**
     * Convenience constructor that uses CAN IDs from
     * {@link frc.robot.Constants.HopperConstants}.
     */
    public Hopper() {
        this(Constants.CanIds.hopperLeftMotorCanID, Constants.CanIds.hopperRightMotorCanID);
    }

    
    public boolean isEmpty() {
        // Placeholder for actual sensor check to determine if hopper is empty
        // This could be implemented with a limit switch, ultrasonic sensor, etc.
        return false; // Assume not empty for now
    }

    /**
     * Stop both hopper motors.
     */
    public void stop() {
        setSpeed(0.0);
    }

    /**
     * Set the percent output for both motors. Values are clamped to [-1, 1].
     *
     * @param speed percent output in range [-1..1]
     */
    public void setSpeed(double speed) {
        double s = MathUtil.clamp(speed, -1.0, 1.0);
        leftMotor.set(s);
        rightMotor.set(s);
    }

    public boolean atMaxPositionL() {
        return leftEncoder.getPosition() >= maxPosition;
    }

    public boolean atMaxPositionR() {
        return rightEncoder.getPosition() >= maxPosition;
    }

    public boolean atMaxPosition() {
        return atMaxPositionL() && atMaxPositionR();
    }

    public boolean atMinPositionL() {
        return leftEncoder.getPosition() <= minPosition;
    }
    public boolean atMinPositionR() {
        return rightEncoder.getPosition() <= minPosition;
    }
    public boolean atMinPosition() {
        return atMinPositionL() && atMinPositionR();
    }

    /**
     * Command that expands the hopper at the default speed while scheduled.
     */
    public Command expandCommand() {
        return new FunctionalCommand(() -> setSpeed(expandSpeed), // init: set speed to expand
                                        () -> { if(atMaxPositionL()) leftMotor.set(0);
                                                if(atMaxPositionR()) rightMotor.set(0); },
                                        (interrupted) -> stop(), // On end (whether interrupted or not), stop the motors
                                        () -> atMaxPosition(), // End condition: stop when hopper reaches max position
                                        this
        );
    }

    public Command retractCommand() {
        return new FunctionalCommand(() -> setSpeed(-expandSpeed), // init: set speed to retract
                                        () -> { if(atMinPositionL()) leftMotor.set(0);
                                                if(atMinPositionR()) rightMotor.set(0); },
                                        (interrupted) -> stop(), // On end (whether interrupted or not), stop the motors
                                        () -> atMinPosition(), // End condition: stop when hopper reaches min position
                                        this
        );
    }
    
    public Command stopCommand() {
        return new InstantCommand(this::stop, this);
    }

    public void loadPreferences() {
        if (Preferences.containsKey(Constants.HopperConstants.expandSpeedKey)) {
            System.out.println("Loading Hopper  values from preferences");
            expandSpeed = Preferences.getDouble(Constants.HopperConstants.expandSpeedKey, HopperConstants.expandSpeedDefault);
            maxPosition = Preferences.getDouble(Constants.HopperConstants.maxPositionKey, HopperConstants.maxPositionDefault);
        } else {
            System.out.println("No hopper prefs found. Using default values");
        }
    }

    public void savePreferences() {
        System.out.println("Saving hopper values to preferences");
        Preferences.setDouble(Constants.HopperConstants.expandSpeedKey, expandSpeed);
        Preferences.setDouble(Constants.HopperConstants.maxPositionKey, maxPosition);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Expand Speed", () -> expandSpeed, (x) -> expandSpeed = x);
        builder.addDoubleProperty("Max Position", () -> maxPosition, (x) -> maxPosition = x);
        builder.addDoubleProperty("Min Position", () -> minPosition, (x) -> minPosition = x);
        builder.addDoubleProperty("Left Motor Position", () -> leftEncoder.getPosition(), null);
        builder.addDoubleProperty("Right Motor Position", () -> rightEncoder.getPosition(), null);
        builder.addBooleanProperty("Save Prefs", ()->false,  (x)->{if(x) savePreferences();});
    }
}
