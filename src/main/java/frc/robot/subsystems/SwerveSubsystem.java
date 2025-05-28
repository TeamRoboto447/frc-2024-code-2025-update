package frc.robot.subsystems;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
// import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/*
 * Example SwerveSubsystem taken from YAGSL-Example 2024 with some value modifications to match our setup
 */
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private final Joystick dJoystick;
    private final XboxController oGamepad;
    public double maximumSpeed = Units.feetToMeters(14.5); // 14.5 is full speed

    public SwerveSubsystem(File directory, Joystick driver, XboxController operator) {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        this.dJoystick = driver;
        this.oGamepad = operator;
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
            // Alternative method if you don't want to supply the conversion factor via JSON
            // files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
            // angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot
                                                 // via angle.

        setupPathPlanner();
        ntInit();
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(6, 0.0, 0.25),
                        // Translation PID constants
                        new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                                swerveDrive.swerveController.config.headingPIDF.i,
                                swerveDrive.swerveController.config.headingPIDF.d),
                        // Rotation PID constants
                        4.5,
                        // Max module speed, in m/s
                        swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig()
                // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public Distance distanceToTarget(Translation2d target) {
        Translation2d robotPos = this.swerveDrive.getPose().getTranslation();
        double measured = robotPos.getDistance(target);
        return edu.wpi.first.units.Units.Meters.of(measured);
    }

    public Rotation2d rotationToTarget(Translation2d target) {
        Translation2d robotPos = this.swerveDrive.getPose().getTranslation();
        double targetRadians = Math.atan2(target.getY() - robotPos.getY(), target.getX() - robotPos.getX());
        return new Rotation2d(targetRadians);
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName       PathPlanner path name.
     * @param setOdomToStart Set the odometry position to the start of the path.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        Optional<Alliance> optional = DriverStation.getAlliance();
        if (optional.isPresent()) {
            Alliance alliance = optional.get();
            if (alliance == Alliance.Red)
                path.flipPath();
        }

        if (setOdomToStart) {
            resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
        }

        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        return AutoBuilder.followPath(path);
    }

    // /**
    // * Construct the swerve drive.
    // *
    // * @param driveCfg SwerveDriveConfiguration for the swerve.
    // * @param controllerCfg Swerve Controller.
    // */
    // public SwerveSubsystem(SwerveDriveConfiguration driveCfg,
    // SwerveControllerConfiguration controllerCfg) {
    // swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
    // }

    /**
     * The primary method for controlling the drivebase. Takes a
     * {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly. Can use either open-loop
     * or closed-loop velocity control for
     * the wheel velocities. Also has field- and robot-relative modes, which affect
     * how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear
     *                      velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards
     *                      the bow (front) and positive y is
     *                      torwards port (left). In field-relative mode, positive x
     *                      is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall
     *                      when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.
     *                      Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for
     *                      robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    private NetworkTable pidTuningPVs;
    private NetworkTableInstance table;
    private NetworkTableEntry distFromTarget;
    private NetworkTableEntry withinRange;
    private void ntInit() {
        table = NetworkTableInstance.getDefault();
        pidTuningPVs = table.getTable("pidTuningPVs");
        distFromTarget = pidTuningPVs.getEntry("distFromTarget");
        withinRange = pidTuningPVs.getEntry("Within Reliable Range");
    }

    @Override
    public void periodic() {
        Translation2d target = FieldConstants.BLUE_SPEAKER;
        Optional<Alliance> maybeAlliance = DriverStation.getAlliance();
        if (maybeAlliance.isPresent()) {
            target = maybeAlliance.get() == Alliance.Blue ? FieldConstants.BLUE_SPEAKER : FieldConstants.RED_SPEAKER;
        }
        double distance = this.distanceToTarget(target).in(edu.wpi.first.units.Units.Meters);
        distFromTarget.setDouble(distance);
        withinRange.setBoolean(distance < 2.75);
        boolean reset = false;
        // if (this.dJoystick.getRawButton(7))
        //     reset = true;
        //     System.out.println("Joystick button 7 pressed");
        // if (this.dJoystick.getRawButton(8))
        //     reset = true;
        //     System.out.println("Joystick button 8 pressed");
        // if (this.dJoystick.getRawButton(9))
        //     reset = true;
        //     System.out.println("Joystick button 9 pressed");
        // if (this.dJoystick.getRawButton(10))
        //     reset = true;
        //     System.out.println("Joystick button 10 pressed");
        // if (this.dJoystick.getRawButton(11))
        //     reset = true;
        //     System.out.println("Joystick button 11 pressed");
        if (this.dJoystick.getRawButton(12)) {
            reset = true;
            System.out.println("Joystick button 12 pressed");
        }
        if (reset) {
            reset = false;
            this.swerveDrive.zeroGyro();
        }
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the imu. CCW
     * positive, not wrapped.
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for
     * speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                headingX,
                headingY,
                getHeading().getRadians(),
                maximumSpeed);
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                angle.getRadians(),
                getHeading().getRadians(),
                maximumSpeed);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    public SwerveDrive getSwerveDrive() {
        return this.swerveDrive;
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return this.swerveDrive.swerveDrivePoseEstimator;
    }
}
