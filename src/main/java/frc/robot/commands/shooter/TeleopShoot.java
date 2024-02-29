package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class TeleopShoot extends Command {
    private final ShooterSubsystem sSubsystem;
    private final DoubleSupplier tSpeedSupplier;
    public TeleopShoot (ShooterSubsystem subsystem, DoubleSupplier speedSupplier) {
        this.sSubsystem = subsystem;
        this.tSpeedSupplier = speedSupplier;
        addRequirements(subsystem); 
    }

    @Override
    public void execute(){
        double speed = this.tSpeedSupplier.getAsDouble();
        this.sSubsystem.spin(speed);
    }
}
