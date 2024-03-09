package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

public class TeleopShoot extends Command {
    private final ShooterSubsystem sSubsystem;
    private final CommandXboxController operatorController;
    public TeleopShoot (ShooterSubsystem subsystem, CommandXboxController operator) {
        this.sSubsystem = subsystem;
        this.operatorController = operator;
        addRequirements(subsystem); 
    }

    @Override
    public void execute(){
        if(this.operatorController.a().getAsBoolean()) {
            this.sSubsystem.spin(0.8);
        } else if(this.operatorController.b().getAsBoolean()) {
            this.sSubsystem.spin(-1);
        } else {
            this.sSubsystem.spin(0);
        }
        if(this.operatorController.back().getAsBoolean()) {
        } else {
            this.sSubsystem.manualAim(-this.operatorController.getRightY() * 0.2);
        }
    }
}
