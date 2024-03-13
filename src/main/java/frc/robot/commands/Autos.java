// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autoScripts.AimRobotBaseAtSpeaker;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public final class Autos {
    public static Command testNamedCommand() {
        return new InstantCommand(() -> {
            System.out.println("Test Succeeded!");
        });
    }

    public static Command aimAtTarget(SwerveSubsystem swerve, ShooterSubsystem shooter) {
        return new AimRobotBaseAtSpeaker(swerve);
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
