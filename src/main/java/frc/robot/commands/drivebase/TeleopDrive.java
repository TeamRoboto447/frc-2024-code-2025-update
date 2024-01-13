package frc.robot.commands.drivebase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

/**
 * Example drive command from YAGSL-Example
 */
public class TeleopDrive extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY, omega;
    private final BooleanSupplier isFieldOriented;
    private final SwerveController controller;

    /**
     * 
     * @param swerve          Swerve Subsystem, so the command can actually control
     *                        speeds
     * @param vX              Horizontal velocity requested
     * @param vY              Vertical velocity requested
     * @param omega           Robot rotation velocity requested
     * @param isFieldOriented Should control be relative to the field? (If false,
     *                        will be relative to the robot instead)
     */
    public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
            BooleanSupplier isFieldOriented) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.isFieldOriented = isFieldOriented;
        this.controller = swerve.getSwerveController();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xVelocity = Math.pow(vX.getAsDouble(), 3);
        double yVelocity = Math.pow(vY.getAsDouble(), 3);
        double angVelocity = Math.pow(omega.getAsDouble(), 3);
        SmartDashboard.putNumber("vX", xVelocity);
        SmartDashboard.putNumber("vY", yVelocity);
        SmartDashboard.putNumber("omega", angVelocity);

        // Drive using raw values.
        swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                angVelocity * controller.config.maxAngularVelocity,
                isFieldOriented.getAsBoolean());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
