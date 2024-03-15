// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoScripts;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AimShooterAtTargetEndless extends Command {
  /** Creates a new AimShooterAtTarget. */
  private final SwerveSubsystem swerve;
  private final ShooterSubsystem shooter;

  public AimShooterAtTargetEndless(SwerveSubsystem swerve, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.shooter = shooter;

    // We don't add the swerve system to requirements as we are only using it to get
    // a reading and are not calling any functions that would move any part
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d target = new Translation2d(0, 0);
    Optional<Alliance> maybeAlliance = DriverStation.getAlliance();
    if (maybeAlliance.isPresent()) {
      target = maybeAlliance.get() == Alliance.Blue ? FieldConstants.BLUE_SPEAKER
          : FieldConstants.RED_SPEAKER;
    }
    double distance = this.swerve.distanceToTarget(target).in(Units.Meters); // Everything on the robot is mapped and
                                                                             // calibrated in metric units
    double angle = this.shooter.getNeededAngleFromDistance(distance);
    this.shooter.autoAim(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
