// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc;

import java.util.Optional;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.RollingAverage;

public class AimAtGenericTranslation extends Command {
    private final SwerveSubsystem swerve;
    private final PIDController headingControl;
    private boolean done;
    private final RollingAverage avgErr;

    /** Creates a new AimAtSpeaker. */
    public AimAtGenericTranslation(SwerveSubsystem swerveSubsystem) {
        this.swerve = swerveSubsystem;
        PIDConstants headingConstants = DrivetrainConstants.autonomousDriveAim;
        this.headingControl = new PIDController(headingConstants.kP, headingConstants.kI, headingConstants.kD);
        this.avgErr = new RollingAverage(50);
        addRequirements(this.swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Translation2d target = new Translation2d(0, 0);
        Optional<Alliance> maybeAlliance = DriverStation.getAlliance();
        if (maybeAlliance.isPresent()) {
            target = maybeAlliance.get() == Alliance.Blue ? FieldConstants.BLUE_SPEAKER : FieldConstants.RED_SPEAKER;
        }
        Translation2d robotPos = swerve.getSwerveDrive().getPose().getTranslation();
        double targetRadians = Math.atan2(target.getY() - robotPos.getY(), target.getX() - robotPos.getX());
        // double angleDegrees = Math.toDegrees(angleRadians);
        double curRadians = this.swerve.getPose().getRotation().getRadians();
        double rotationSpeed = -this.headingControl.calculate(curRadians,
                targetRadians);

        this.avgErr.add(curRadians - targetRadians);
        SmartDashboard.putNumberArray("Avg Error", new Double[] {
                Math.abs(this.avgErr.getAvg()),
                curRadians,
                targetRadians
        });

        if (Math.abs(this.avgErr.getAvg()) < 0.2) {
            rotationSpeed = 0;
            done = true;
        }
        this.swerve.drive(new Translation2d(0, 0),
                rotationSpeed * (this.swerve.getSwerveController().config.maxAngularVelocity * 0.5), true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (done) {
            done = false;
            this.avgErr.reset();
            return true;
        }
        return false;
    }
}
