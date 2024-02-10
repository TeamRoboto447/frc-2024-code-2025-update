package frc.robot.commands.testing;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoTestingSubsystem;

public class ServoTestingCommand extends Command {
    private final ServoTestingSubsystem subsystem;
    private final DoubleSupplier supplier;
    public ServoTestingCommand(ServoTestingSubsystem subsystem, DoubleSupplier speedSupplier) {
        this.subsystem = subsystem;
        this.supplier = speedSupplier;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        this.subsystem.setPosition(this.supplier.getAsDouble());
    }
}
