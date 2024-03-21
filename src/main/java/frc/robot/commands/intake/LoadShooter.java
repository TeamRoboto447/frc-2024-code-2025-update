// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeStatus;

public class LoadShooter extends Command {
  private final IntakeSubsystem subsystem;
  public LoadShooter(IntakeSubsystem intake) {
    this.subsystem = intake;
    addRequirements(this.subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.subsystem.raiseIntake();
    double speed = this.subsystem.getState() == IntakeStatus.RAISED ? 1 : 0;
    this.subsystem.load(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.subsystem.idleIntake();
    this.subsystem.load(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
