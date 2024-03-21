package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

public class TeleopIndex extends Command {
    private final IntakeSubsystem subsystem;
    private final XboxController operatorController;

    public TeleopIndex(IntakeSubsystem iSubsystem, CommandXboxController operatorController) {
        this.subsystem = iSubsystem;
        this.operatorController = operatorController.getHID();
        addRequirements(this.subsystem);
    }
    
    @Override
    public void execute() {
        // this.subsystem.liftManual(-operatorController.getLeftY() / 8);
        this.subsystem.load(operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis());
        this.subsystem.manualIntake(operatorController.getXButton() ? 1 : operatorController.getYButton() ? -1 : 0);
        double leftStickY = this.operatorController.getLeftY();
        if(Math.abs(leftStickY) <= 0.5) this.subsystem.idleIntake();
        if(leftStickY < -0.5) this.subsystem.raiseIntake();
        if(leftStickY > 0.5) this.subsystem.lowerIntake();
    }
}
