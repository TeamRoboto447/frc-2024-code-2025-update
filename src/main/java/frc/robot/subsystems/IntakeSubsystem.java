package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SwerveSubsystem driveSystem;

    private final CANSparkMax left;
    private final PIDController leftController;
    private final CANSparkMax front;
    private final PIDController frontController;
    private final CANSparkMax right;
    private final PIDController rightController;
    private final CANSparkMax loader;
    private final Servo platformLifter0;
    private final Servo platformLifter1;

    private boolean shouldbeLifted = false;

    public IntakeSubsystem(SwerveSubsystem driveSystem) {
        this.driveSystem = driveSystem;

        this.left = new CANSparkMax(IntakeConstants.leftMotorId, MotorType.kBrushless);
        this.leftController = new PIDController(
                IntakeConstants.leftControllerVals[0],
                IntakeConstants.leftControllerVals[1],
                IntakeConstants.leftControllerVals[2]);
        this.front = new CANSparkMax(IntakeConstants.frontMotorId, MotorType.kBrushless);
        this.frontController = new PIDController(
                IntakeConstants.frontControllerVals[0],
                IntakeConstants.frontControllerVals[1],
                IntakeConstants.frontControllerVals[2]);
        this.right = new CANSparkMax(IntakeConstants.rightMotorId, MotorType.kBrushless);
        this.rightController = new PIDController(
                IntakeConstants.rightControllerVals[0],
                IntakeConstants.rightControllerVals[1],
                IntakeConstants.rightControllerVals[2]);
        this.loader = new CANSparkMax(IntakeConstants.loaderMotorId, MotorType.kBrushless);

        this.platformLifter0 = new Servo(IntakeConstants.lifter0Channel);
        this.platformLifter1 = new Servo(IntakeConstants.lifter1Channel);
    }

    public boolean isIntakeUp() {
        return this.shouldbeLifted; // TODO: Detect if lifted rather than just trusting the variable
    }

    public void raiseIntake() {
        this.shouldbeLifted = true;
    }

    public void lowerIntake() {
        this.shouldbeLifted = false;
    }

    public void setLoaderSpeed(double speed) {
        this.loader.set(speed);
    }

    @Override
    public void periodic() {
        runLifterServos();
        runIntake();
    }

    private void runLifterServos() {
        if (this.shouldbeLifted) {
            this.platformLifter0.setPosition(1);
            this.platformLifter1.setPosition(1);
        } else {
            this.platformLifter0.setPosition(0);
            this.platformLifter1.setPosition(0);
        }
    }

    private void runIntake() {
        if (!isIntakeUp()) {
            this.left.set(1);
            this.front.set(1);
            this.right.set(1);
            
            this.leftController.reset();
            this.frontController.reset();
            this.rightController.reset();
        } else {
            // TODO: logic for note movement while intake is out of the way
            ChassisSpeeds velocities = this.driveSystem.getRobotVelocity();
            double xSpeed = velocities.vxMetersPerSecond;
            double ySpeed = velocities.vyMetersPerSecond;

            // Untested!
            this.left.set(this.leftController.calculate(getLeftSpeed(), xSpeed));
            this.front.set(this.frontController.calculate(getFrontSpeed(), ySpeed));
            this.right.set(this.rightController.calculate(getRightSpeed(), xSpeed));
        }
    }

    private double getLeftSpeed() {
        return rpmToMps(this.left.getEncoder().getVelocity());
    }

    private double getFrontSpeed() {
        return rpmToMps(this.front.getEncoder().getVelocity());
    }

    private double getRightSpeed() {
        return rpmToMps(this.right.getEncoder().getVelocity());
    }

    private double rpmToMps(double RPM) {
        double RPS = RPM / 60;
        double speedInMeterPerSecond = RPS * IntakeConstants.intakeWheelCircunferenceMeters;
        return speedInMeterPerSecond;
    }

}
