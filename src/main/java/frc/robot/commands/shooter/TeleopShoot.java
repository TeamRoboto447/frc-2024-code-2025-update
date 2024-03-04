package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class TeleopShoot extends Command {
    private final ShooterSubsystem sSubsystem;
    private final DoubleSupplier tSpeedSupplier;
    private final DoubleSupplier aimSpeedSupplier;
    public TeleopShoot (ShooterSubsystem subsystem, DoubleSupplier speedSupplier, DoubleSupplier aimSpeedSupplier) {
        this.sSubsystem = subsystem;
        this.tSpeedSupplier = speedSupplier;
        this.aimSpeedSupplier = aimSpeedSupplier;
        addRequirements(subsystem); 
    }

    @Override
    public void execute(){
        double speed = this.tSpeedSupplier.getAsDouble();
        this.sSubsystem.spin(speed);
        this.sSubsystem.manualAim(this.aimSpeedSupplier.getAsDouble());
    }
}
