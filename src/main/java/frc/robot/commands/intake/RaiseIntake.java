// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeStatus;

public class RaiseIntake extends Command {
  /** Creates a new RaiseIntake. */
  private final IntakeSubsystem subsystem;
  private final boolean isEndless;
  public RaiseIntake(IntakeSubsystem intake, boolean endless) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = intake;
    this.isEndless = endless;
    addRequirements(this.subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.subsystem.raiseIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.subsystem.getState() == IntakeStatus.RAISED && !this.isEndless;
  }
}
