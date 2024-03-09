package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class TeleopIndex extends Command {
    private final IntakeSubsystem subsystem;
    private final DoubleSupplier liftSpeed;
    private final DoubleSupplier loadSpeed;
    private final DoubleSupplier intakeSpeed;

    public TeleopIndex(IntakeSubsystem iSubsystem, DoubleSupplier lifterSpeed, DoubleSupplier loaderSpeed, DoubleSupplier intakeSpeed) {
        this.subsystem = iSubsystem;
        this.liftSpeed = lifterSpeed;
        this.loadSpeed = loaderSpeed;
        this.intakeSpeed = intakeSpeed;
        addRequirements(this.subsystem);
    }
    
    @Override
    public void execute() {
        this.subsystem.liftManual(this.liftSpeed.getAsDouble());
        this.subsystem.load(this.loadSpeed.getAsDouble());
        this.subsystem.manualIntake(this.intakeSpeed.getAsDouble());
    }
}
