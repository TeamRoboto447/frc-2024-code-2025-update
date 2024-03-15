package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

public class TeleopShoot extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final XboxController operatorController;

    public TeleopShoot(ShooterSubsystem subsystem, CommandXboxController operator) {
        this.shooterSubsystem = subsystem;
        this.operatorController = operator.getHID();
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (this.operatorController.getAButton()) {
            this.shooterSubsystem.spin(1);
        } else if (this.operatorController.getBButton()) {
            this.shooterSubsystem.spin(0.2);
        } else if (this.operatorController.getBackButton()) {
            this.shooterSubsystem.spin(-1);
        } else {
            this.shooterSubsystem.spin(0);
        }
        this.shooterSubsystem.manualAim(-this.operatorController.getRightY() * 0.2);
    }
}
