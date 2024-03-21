// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeStatus;

public class RunIntake extends Command {
  private final IntakeSubsystem subsystem;
  public RunIntake(IntakeSubsystem intake) {
    this.subsystem = intake;
    addRequirements(this.subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.subsystem.lowerIntake();
    double speed = this.subsystem.getState() == IntakeStatus.LOWERED || this.subsystem.getState() == IntakeStatus.STALLED_AT_BOTTOM ? 1 : 0;
    this.subsystem.manualIntake(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.subsystem.idleIntake();
    this.subsystem.manualIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
