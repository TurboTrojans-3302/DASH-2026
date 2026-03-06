// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.rangeRPMtable;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetRangeAndAim extends Command {
  private final DriveTrain driveTrain;
  private final Navigation navigation;
  private final Shooter shooter;

  private static final double kAimToleranceDegrees = 2.0;

  /** Creates a new AutoAim. */
  public SetRangeAndAim(DriveTrain driveTrain, Navigation navigation, Shooter shooter) {
    this.driveTrain = driveTrain;
    this.navigation = navigation;
    this.shooter = shooter;
    addRequirements(driveTrain, navigation, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setShooterSpeedForDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveHeading(new Translation2d(0.0, 0.0), navigation.getHeadingToTarget());
    setShooterSpeedForDistance(); // just in case distance has changed a bit
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MathUtil.isNear(navigation.getHeadingToTarget().getRadians(), driveTrain.getHeading().getRadians(),
                           Math.toRadians(kAimToleranceDegrees),
                           0.0, (2 * Math.PI))
           && shooter.isReady();
  }

  private void setShooterSpeedForDistance() {
    double distance = navigation.getDxToHubCenter();
    double rpm = rangeRPMtable.get(distance);
    shooter.setRPMsetpoint(rpm);
  }
}
