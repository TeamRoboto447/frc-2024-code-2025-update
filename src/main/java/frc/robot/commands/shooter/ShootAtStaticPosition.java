// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeStatus;

public class ShootAtStaticPosition extends Command {
  /** Creates a new Shoot. */
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private final Timer timer;

  public ShootAtStaticPosition(ShooterSubsystem shooterSubsystem, IntakeSubsystem intake) {
    this.shooter = shooterSubsystem;
    this.intake = intake;
    this.timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intake);
  }

  private boolean shooting = false;
  private boolean done = false;
  private double target;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer.reset();
    this.shooting = false;
    this.done = false;
    this.target = 55.9;
    this.intake.load(0);
    this.shooter.spin(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.spin(0.75);
    this.intake.raiseIntake();
    boolean ready = this.shooter.autoAim(this.target) && this.intake.getState() == IntakeStatus.RAISED;
    SmartDashboard.putBoolean("Auto Shot Ready", ready);
    if (ready && !shooting) {
      this.timer.start();
      this.shooting = true;
      this.intake.load(0.75);
    }
    if (shooting && this.timer.advanceIfElapsed(3)) {
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
