// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeStatus;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  private final ShooterSubsystem shooter;
  private final SwerveSubsystem swerve;
  private final IntakeSubsystem intake;
  private final Timer timer;

  public Shoot(ShooterSubsystem shooterSubsystem, SwerveSubsystem swerve, IntakeSubsystem intake) {
    this.shooter = shooterSubsystem;
    this.swerve = swerve;
    this.intake = intake;
    this.timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intake);
  }

  private boolean shooting = false;
  private boolean done = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer.reset();
    this.shooting = false;
    this.done = false;
    this.intake.load(0);
    this.shooter.spin(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.spin(0.75);
    this.intake.raiseIntake();
    Translation2d target = new Translation2d(0, 0);
    Optional<Alliance> maybeAlliance = DriverStation.getAlliance();
    if (maybeAlliance.isPresent()) {
      target = maybeAlliance.get() == Alliance.Blue ? FieldConstants.BLUE_SPEAKER
          : FieldConstants.RED_SPEAKER;
    }
    double distance = this.swerve.distanceToTarget(target).in(Units.Meters); // Everything on the robot is mapped and
                                                                             // calibrated in metric units
    double angle = this.shooter.getNeededAngleFromDistance(distance);
    boolean ready = this.shooter.autoAim(angle) && this.intake.getState() == IntakeStatus.RAISED;
    if (ready && !shooting) {
      this.timer.start();
      this.shooting = true;
      this.intake.load(0.75);
    }
    if (shooting && this.timer.advanceIfElapsed(2.5)) {
      this.done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intake.idleIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.done;
  }
}
