// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAtCurrPosition extends Command {
  /** Creates a new Shoot. */
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private final Timer timer;

  public ShootAtCurrPosition(ShooterSubsystem shooterSubsystem, IntakeSubsystem intake) {
    this.shooter = shooterSubsystem;
    this.intake = intake;
    this.timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intake);
  }

  private boolean timerStarted = false;
  private boolean atSpeed = false;
  private boolean done = false;
  private double target = 49.98;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer.reset();
    this.timerStarted = false;
    this.done = false;
    this.atSpeed = false;
    double curAngle = this.shooter.getAngle();
    this.target = curAngle > 0 ? curAngle : target; // If angle is valid, set the angle. Otherwise use default
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.spin(1);
    this.intake.raiseIntake();
    boolean onTarget = this.shooter.autoAim(this.target);
    if (onTarget && !timerStarted) {
      this.timer.start();
      this.timerStarted = true;
    }
    if (timerStarted && this.timer.advanceIfElapsed(2.5)) {
      if (atSpeed)
        this.done = true;
      if (!atSpeed) {
        this.atSpeed = true;
        this.intake.load(0.75);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.done;
  }
}
