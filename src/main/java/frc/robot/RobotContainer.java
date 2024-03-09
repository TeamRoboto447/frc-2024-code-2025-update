// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.climber.ClimbTeleop;
import frc.robot.commands.drivebase.TeleopDrive;
import frc.robot.commands.intake.TeleopIndex;
import frc.robot.commands.shooter.TeleopShoot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
    private final boolean usingJoystick = true; // Set to false if using gamepad
    private final double maxAllowedSpeedRange = 0.5; // percentage of max speed (inputs are multiplied by this number)
    private final double turnSpeedPercentage =  0.75; // percentage of max turn speed to allow

    private final SendableChooser<Command> autoChooser;
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    private final PoseEstimatorSubsystem poseSubsystem = new PoseEstimatorSubsystem(
            () -> drivebase.getSwerveDrive().getYaw(), () -> drivebase.getSwerveDrive().getModulePositions(),
            drivebase);

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(drivebase);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    CommandJoystick driverJoystick = new CommandJoystick(ControllerConstants.kDriverControllerPort);
    CommandXboxController driverGamepad = new CommandXboxController(ControllerConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(
            ControllerConstants.kOperatorControllerPort);

    private final TeleopIndex indexerCommand = new TeleopIndex(
            intakeSubsystem,
            () -> (-operatorController.getLeftY()) / 4,
            () -> operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis(),
            () -> operatorController.x().getAsBoolean() ? 1 : operatorController.y().getAsBoolean() ? -1 : 0 // if x
                                                                                                             // return
                                                                                                             // 1, else
                                                                                                             // if y
                                                                                                             // return
                                                                                                             // -1, else
                                                                                                             // return 0
    );
    private final TeleopShoot shooterCommand = new TeleopShoot(this.shooterSubsystem, this.operatorController);

    private final DoubleSupplier climbSpeedSupplierJoystick = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            boolean up = driverJoystick.button(6).getAsBoolean();
            boolean down = driverJoystick.button(4).getAsBoolean();
            double speed = 1;
            return up ? speed : down ? -speed : 0; // if up return speed else if down return -speed else return 0
        }
    };
    private final DoubleSupplier climbSpeedSupplierGamepad = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            boolean up = driverGamepad.a().getAsBoolean();
            boolean down = driverGamepad.b().getAsBoolean();
            double speed = 1;
            return up ? speed : down ? -speed : 0; // if up return speed else if down return -speed else return 0
        }
    };
    private final ClimbTeleop climberCommand = new ClimbTeleop(climberSubsystem,
            usingJoystick ? climbSpeedSupplierJoystick : climbSpeedSupplierGamepad);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure Named Commands
        NamedCommands.registerCommand("Test Command", Autos.testNamedCommand());

        // Configure the trigger bindings
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("Test Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        TeleopDrive closedFieldRel = null;
        if (usingJoystick) {
            closedFieldRel = new TeleopDrive(drivebase,
                    () -> -MathUtil.applyDeadband(driverJoystick.getY() * maxAllowedSpeedRange,
                            ControllerConstants.Y_DEADBAND),
                    () -> -MathUtil.applyDeadband(driverJoystick.getX() * maxAllowedSpeedRange,
                            ControllerConstants.X_DEADBAND),
                    () -> -MathUtil.applyDeadband(
                            (driverJoystick.button(1).getAsBoolean() ? driverJoystick.getZ() * 0.5 : 0),
                            ControllerConstants.Z_DEADBAND),
                    () -> true);
        } else {
            closedFieldRel = new TeleopDrive(drivebase,
                    () -> -MathUtil.applyDeadband(driverGamepad.getLeftY() *
                            maxAllowedSpeedRange,
                            ControllerConstants.Y_DEADBAND),
                    () -> -MathUtil.applyDeadband(driverGamepad.getLeftX() *
                            maxAllowedSpeedRange,
                            ControllerConstants.X_DEADBAND),
                    () -> -MathUtil.applyDeadband(driverGamepad.getRightX() * turnSpeedPercentage,
                            ControllerConstants.Z_DEADBAND),
                    () -> true);
        }

        // TeleopDrive closedFieldRel = new TeleopDrive(drivebase,
        // () -> MathUtil.applyDeadband(0,
        // ControllerConstants.Y_DEADBAND),
        // () -> MathUtil.applyDeadband(0,
        // ControllerConstants.X_DEADBAND),
        // () -> MathUtil.applyDeadband(0,
        // ControllerConstants.Z_DEADBAND),
        // () -> true);

        drivebase.setDefaultCommand(closedFieldRel);
        shooterSubsystem.setDefaultCommand(shooterCommand);
        intakeSubsystem.setDefaultCommand(indexerCommand);
        climberSubsystem.setDefaultCommand(climberCommand);
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
    private void configureBindings() {
    }

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
        drivebase.setMotorBrake(brake);
    }

    public void onAllianceChanged(Alliance currentAlliance) {
        poseSubsystem.setAlliance(currentAlliance);
    }
}
