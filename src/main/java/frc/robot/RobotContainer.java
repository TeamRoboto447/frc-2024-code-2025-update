// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.drivebase.TeleopDrive;
import frc.robot.commands.shooter.TeleopShoot;
import frc.robot.commands.testing.ServoTestingCommand;
import frc.robot.subsystems.ServoTestingSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    private final PoseEstimatorSubsystem poseSubsystem = new PoseEstimatorSubsystem(
            () -> drivebase.getSwerveDrive().getYaw(), () -> drivebase.getSwerveDrive().getModulePositions(),
            drivebase);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ServoTestingSubsystem testingSubsystem = new ServoTestingSubsystem();

    CommandJoystick driverController = new CommandJoystick(ControllerConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(
            ControllerConstants.kOperatorControllerPort);

    private final DoubleSupplier speedSupplier = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            // double speed = (-driverController.getRawAxis(3)) / 2; // Half speed max
            double speed = -((driverController.getRawAxis(3) + 1) / 2); // Full speed max

            if (Math.abs(speed) < 0.15) {
                speed = 0;
            }

            if (!operatorController.a().getAsBoolean() && !operatorController.b().getAsBoolean())
                speed = 0;

            if (operatorController.b().getAsBoolean())
                speed = -speed;
            return speed;
        }
    };

    private final TeleopShoot shooterCommand = new TeleopShoot(shooterSubsystem, speedSupplier);
    private final ServoTestingCommand testCommand = new ServoTestingCommand(testingSubsystem, () -> {
        double position = (driverController.getRawAxis(3) + 1)/2;
        return position;
    });

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure Named Commands
        NamedCommands.registerCommand("Test Command", Autos.testNamedCommand());

        // Configure the trigger bindings
        configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        TeleopDrive closedFieldRel = new TeleopDrive(drivebase,
                () -> MathUtil.applyDeadband(driverController.getY(),
                        ControllerConstants.Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverController.getX(),
                        ControllerConstants.X_DEADBAND),
                () -> MathUtil.applyDeadband(-driverController.getZ(),
                        ControllerConstants.Z_DEADBAND),
                () -> true);

        drivebase.setDefaultCommand(closedFieldRel);
        shooterSubsystem.setDefaultCommand(shooterCommand);
        testingSubsystem.setDefaultCommand(testCommand);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setDriveMode() {
        // drivebase.setDefaultCommand();
    }

    public void setMotorBrake(boolean brake) {
        // drivebase.setMotorBrake(brake);
    }

    public void onAllianceChanged(Alliance currentAlliance) {
        poseSubsystem.setAlliance(currentAlliance);
    }
}
