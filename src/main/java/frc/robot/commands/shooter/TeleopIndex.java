package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class TeleopIndex extends Command {
    private final IntakeSubsystem subsystem;

    public TeleopIndex(IntakeSubsystem iSubsystem) {
        this.subsystem = iSubsystem;
        addRequirements(this.subsystem);
    }
    
    @Override
    public void execute() {
        
    }
}
