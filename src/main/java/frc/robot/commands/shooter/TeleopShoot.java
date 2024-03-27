package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

public class TeleopShoot extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final XboxController operatorController;
    private final Joystick driverController;

    public TeleopShoot(ShooterSubsystem subsystem, CommandXboxController operator, CommandJoystick driver) {
        this.shooterSubsystem = subsystem;
        this.operatorController = operator.getHID();
        this.driverController = driver.getHID();
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        boolean isB = this.operatorController.getBButton();
        if (this.operatorController.getAButton()) {
            this.shooterSubsystem.spin(0.7);
        } else if (isB) {
            this.shooterSubsystem.autoAim(54.87375107427691);
            double speed = 0.37007874250412;
            this.shooterSubsystem.spinDifferently(speed, speed * 0.25);
        } else if (this.operatorController.getBackButton()) {
            this.shooterSubsystem.spin(-0.2);
        }
        else if(this.operatorController.getPOV() == 270) {
            double speed = (-driverController.getRawAxis(3) + 1)/2;
            this.shooterSubsystem.spinDifferently(speed, speed * 0.25);
            SmartDashboard.putNumber("Amp Speed", speed);
        }
        else if(this.operatorController.getPOV() == 90) {
            double speed = (-driverController.getRawAxis(3) + 1)/2;
            this.shooterSubsystem.spinDifferently(speed, speed);
            SmartDashboard.putNumber("Trap Speed", speed);
        }
        
        else {
            this.shooterSubsystem.spin(0);
        }
        double aimSpeed = MathUtil.applyDeadband(this.operatorController.getRightY(), 0.25);
        if(!isB)
            this.shooterSubsystem.manualAim(-aimSpeed * 0.2);
    }
}
