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
import frc.robot.commands.testing.ServoTestingCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ServoTestingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
    private final boolean driverUsingJoystick = true; // Set to false if using gamepad
    private final double maxAllowedSpeedRange = 0.5; // percentage of max speed (inputs are multiplied by this number)
    private final double turnSpeedPercentage = 0.5; // percentage of max turn speed to allow
    private final double adjustSpeed = 0.15; // percentage of the max speed to move when using fine adjustments
    private final double adjustTurnSpeed = 0.15; // percentage of the max speed to move when using fine adjustments

    private final SendableChooser<Command> autoChooser;
    // The robot's subsystems and commands are defined here...

    private final CommandJoystick commandDriverJoystick = new CommandJoystick(
            ControllerConstants.kDriverControllerPort);
    private final Joystick basicDriverJoystick = commandDriverJoystick.getHID();
    private final CommandXboxController commandDriverGamepad = new CommandXboxController(
            ControllerConstants.kDriverControllerPort);
    private final XboxController basicDriverGamepad = commandDriverGamepad.getHID();

    private final CommandXboxController commandOperatorController = new CommandXboxController(
            ControllerConstants.kOperatorControllerPort);
    private final XboxController basicOperatorController = commandOperatorController.getHID();

    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"), basicDriverJoystick, basicOperatorController);

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(drivebase);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    private final TeleopIndex indexerCommand = new TeleopIndex(intakeSubsystem, commandOperatorController);

    private final TeleopShoot shooterCommand = new TeleopShoot(this.shooterSubsystem, this.commandOperatorController);

    private final DoubleSupplier climbSpeedSupplierJoystick = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            boolean up = basicDriverJoystick.getRawButton(6) || basicOperatorController.getPOV() == 0;
            boolean down = basicDriverJoystick.getRawButton(5) || basicOperatorController.getPOV() == 180;
            double speed = 1;
            return up ? speed : down ? -speed : 0;
        }
    };
    private final DoubleSupplier climbSpeedSupplierGamepad = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            boolean up = basicDriverGamepad.getAButton() || basicOperatorController.getPOV() == 0;
            boolean down = basicDriverGamepad.getBButton() || basicOperatorController.getPOV() == 180;
            double speed = 1;
            return up ? speed : down ? -speed : 0;
        }
    };
    private final ClimbTeleop climberCommand = new ClimbTeleop(climberSubsystem,
            driverUsingJoystick ? climbSpeedSupplierJoystick : climbSpeedSupplierGamepad);

    private double yDirFromPOV(int pov) {
        if (pov > 270 || (pov < 90 && pov > -1))
            return -1; // forward
        if (pov > 90 && pov < 270)
            return 1; // backward
        return 0;
    }

    private double xDirFromPOV(int pov) {
        if (pov > 0 && pov < 180)
            return 1; // right
        if (pov > 180 && pov < 360)
            return -1; // left
        return 0;
    }

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

        new PoseEstimatorSubsystem(() -> drivebase.getSwerveDrive().getYaw(),
                () -> drivebase.getSwerveDrive().getModulePositions(), drivebase);

        TeleopDrive closedFieldRel = null;
        DoubleSupplier ySpeed = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                double speed = 0;
                speed = yDirFromPOV(commandDriverJoystick.getHID().getPOV()) * adjustSpeed;
                if (speed == 0)
                    speed = MathUtil.applyDeadband(commandDriverJoystick.getY() * maxAllowedSpeedRange,
                            ControllerConstants.Y_DEADBAND);
                return speed;
            }
        };

        DoubleSupplier xSpeed = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                double speed = 0;
                speed = xDirFromPOV(commandDriverJoystick.getHID().getPOV()) * adjustSpeed;
                if (speed == 0)
                    speed = MathUtil.applyDeadband(commandDriverJoystick.getX() * maxAllowedSpeedRange,
                            ControllerConstants.X_DEADBAND);
                return speed;
            }
        };

        DoubleSupplier turnSpeed = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                double speed = 0;
                if (basicDriverJoystick.getRawButton(1)) {
                    speed = MathUtil.applyDeadband((commandDriverJoystick.getZ() * turnSpeedPercentage),
                            ControllerConstants.Z_DEADBAND);
                } else {
                    if (basicDriverGamepad.getRawButton(4))
                        speed = adjustTurnSpeed;
                    if (basicDriverGamepad.getRawButton(3))
                        speed = -adjustTurnSpeed;
                }
                return speed;
            }
        };

        closedFieldRel = new TeleopDrive(drivebase,
                ySpeed,
                xSpeed,
                turnSpeed);

        drivebase.setDefaultCommand(closedFieldRel);
        shooterSubsystem.setDefaultCommand(shooterCommand);
        intakeSubsystem.setDefaultCommand(indexerCommand);
        climberSubsystem.setDefaultCommand(climberCommand);
        // ServoTestingSubsystem servos = new ServoTestingSubsystem();
        // servos.setDefaultCommand(new ServoTestingCommand(servos, () ->
        // this.commandDriverJoystick.getRawAxis(3)));
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
        commandOperatorController.leftBumper().onTrue(Autos.aimAtTarget(drivebase, shooterSubsystem));
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
}
