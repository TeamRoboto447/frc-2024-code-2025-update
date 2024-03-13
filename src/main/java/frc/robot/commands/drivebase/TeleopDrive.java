package frc.robot.commands.drivebase;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

/**
 * Example drive command from YAGSL-Example
 */
public class TeleopDrive extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY, omega;
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
    public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
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
        Optional<Alliance> maybeAlliance = DriverStation.getAlliance();
        double xVelocity;
        double yVelocity;
        if (maybeAlliance.isPresent()) {
            if (maybeAlliance.get() == Alliance.Blue) {
                xVelocity = -vX.getAsDouble(); // Math.pow(vX.getAsDouble(), 3);
                yVelocity = -vY.getAsDouble(); // Math.pow(vY.getAsDouble(), 3);
            } else {
                xVelocity = vX.getAsDouble(); // Math.pow(vX.getAsDouble(), 3);
                yVelocity = vY.getAsDouble(); // Math.pow(vY.getAsDouble(), 3);
            }
        } else {
            xVelocity = vX.getAsDouble();
            yVelocity = vY.getAsDouble();
        }
        double angVelocity = omega.getAsDouble(); // Math.pow(omega.getAsDouble(), 3);
        // SmartDashboard.putNumber("vX", xVelocity);
        // SmartDashboard.putNumber("vY", yVelocity);
        // SmartDashboard.putNumber("omega", angVelocity);

        // Drive using raw values.
        Translation2d translation = new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed);
        translation = translation.rotateBy(this.swerve.getHeading().times(-1));
        swerve.drive(translation,
                angVelocity * controller.config.maxAngularVelocity,
                false);
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
