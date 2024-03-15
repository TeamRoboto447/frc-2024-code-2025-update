// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.autoScripts.AimRobotBaseAtSpeaker;
import frc.robot.commands.autoScripts.AimShooterAtTarget;
import frc.robot.commands.autoScripts.AimShooterAtTargetEndless;
import frc.robot.commands.autoScripts.Shoot;
import frc.robot.commands.autoScripts.ShootAt49Point9;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public final class Autos {
    public static Command testNamedCommand() {
        return new InstantCommand(() -> {
            System.out.println("Test Succeeded!");
        });
    }

    public static Command aimAtTarget(SwerveSubsystem swerve, ShooterSubsystem shooter) {
        return new ParallelCommandGroup(
            new AimRobotBaseAtSpeaker(swerve),
            new AimShooterAtTarget(swerve, shooter));
    }

    public static Command shoot(SwerveSubsystem swerve, ShooterSubsystem shooter, IntakeSubsystem intake) {
        return new ParallelCommandGroup(
            new AimRobotBaseAtSpeaker(swerve),
            new Shoot(shooter, swerve, intake)
        );
    }

    public static Command shootAtStart(SwerveSubsystem swerve, ShooterSubsystem shooter, IntakeSubsystem intake) {
        return new ParallelCommandGroup(
            // new AimRobotBaseAtSpeaker(swerve),
            new ShootAt49Point9(shooter, swerve, intake)
        );
    }

    public static Command keepShooterAimedEndless(SwerveSubsystem swerve, ShooterSubsystem shooter) {
        return new AimShooterAtTargetEndless(swerve, shooter);
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
