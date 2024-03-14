package frc.robot.subsystems;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkLimitSwitch.Type;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    // private final SwerveSubsystem driveSystem;
    // private final PIDController leftController;
    // private final PIDController frontController;
    // private final PIDController rightController;

    private final CANSparkMax left;
    private final CANSparkMax front;
    private final CANSparkMax right;
    private final CANSparkMax loader;

    private final CANSparkMax lifterOne;
    // private final SparkLimitSwitch downLimitOne;
    private final SparkLimitSwitch upLimitOne;
    private final RelativeEncoder liftEncoderOne;
    private final CANSparkMax lifterTwo;
    // private final SparkLimitSwitch downLimitTwo;
    private final SparkLimitSwitch upLimitTwo;
    private final RelativeEncoder liftEncoderTwo;
    private final CANSparkMax lifterThree;
    // private final SparkLimitSwitch downLimitThree;
    private final SparkLimitSwitch upLimitThree;
    private final RelativeEncoder liftEncoderThree;

    // private boolean shouldbeLifted = false;
    // private boolean liftManualControl = false;

    private boolean oneHasResetSinceSwitch = false;
    private boolean twoHasResetSinceSwitch = false;
    private boolean threeHasResetSinceSwitch = false;

    public IntakeSubsystem(SwerveSubsystem driveSystem) {
        // this.driveSystem = driveSystem;
        // this.leftController = new PIDController(
        // IntakeConstants.leftControllerVals[0],
        // IntakeConstants.leftControllerVals[1],
        // IntakeConstants.leftControllerVals[2]);

        // this.frontController = new PIDController(
        // IntakeConstants.frontControllerVals[0],
        // IntakeConstants.frontControllerVals[1],
        // IntakeConstants.frontControllerVals[2]);
        // this.rightController = new PIDController(
        // IntakeConstants.rightControllerVals[0],
        // IntakeConstants.rightControllerVals[1],
        // IntakeConstants.rightControllerVals[2]);

        this.left = new CANSparkMax(IntakeConstants.leftMotorId, MotorType.kBrushless);
        this.left.setInverted(false);
        this.setPeriods(this.left);

        this.front = new CANSparkMax(IntakeConstants.frontMotorId, MotorType.kBrushless);
        this.front.setInverted(false);
        this.setPeriods(this.front);

        this.right = new CANSparkMax(IntakeConstants.rightMotorId, MotorType.kBrushless);
        this.right.setInverted(true);
        this.setPeriods(this.right);

        this.loader = new CANSparkMax(IntakeConstants.loaderMotorId, MotorType.kBrushless);

        this.lifterOne = new CANSparkMax(IntakeConstants.lifterOne, MotorType.kBrushless);
        this.lifterOne.setInverted(false);
        this.lifterOne.setIdleMode(IdleMode.kBrake);
        this.setPeriods(this.lifterOne);
        this.lifterTwo = new CANSparkMax(IntakeConstants.lifterTwo, MotorType.kBrushless);
        this.lifterTwo.setInverted(false);
        this.lifterOne.setIdleMode(IdleMode.kBrake);
        this.setPeriods(this.lifterTwo);
        this.lifterThree = new CANSparkMax(IntakeConstants.lifterThree, MotorType.kBrushless);
        this.lifterThree.setInverted(false);
        this.lifterOne.setIdleMode(IdleMode.kBrake);
        this.setPeriods(this.lifterThree);

        this.upLimitOne = lifterOne.getForwardLimitSwitch(Type.kNormallyOpen);
        this.upLimitTwo = lifterTwo.getForwardLimitSwitch(Type.kNormallyOpen);
        this.upLimitThree = lifterThree.getForwardLimitSwitch(Type.kNormallyOpen);

        // this.downLimitOne = lifterOne.getReverseLimitSwitch(Type.kNormallyOpen);
        // this.downLimitTwo = lifterTwo.getReverseLimitSwitch(Type.kNormallyOpen);
        // this.downLimitThree = lifterThree.getReverseLimitSwitch(Type.kNormallyOpen);

        this.liftEncoderOne = lifterOne.getEncoder();
        this.liftEncoderTwo = lifterTwo.getEncoder();
        this.liftEncoderThree = lifterThree.getEncoder();
    }

    private void setPeriods(CANSparkMax sparkMax) {
        sparkMax.setCANTimeout(500);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);
    }

    public boolean oneAtUpperLimit() {
        return this.upLimitOne.isPressed();
    }

    public boolean twoAtUpperLimit() {
        return this.upLimitTwo.isPressed();
    }

    public boolean threeAtUpperLimit() {
        return this.upLimitThree.isPressed();
    }

    public boolean allAtUpperLimit() {
        return oneAtUpperLimit() && twoAtUpperLimit() && threeAtUpperLimit();
    }

    public boolean oneAtLowerLimit() {
        return this.liftEncoderOne.getPosition() <= -450;
    }

    public boolean twoAtLowerLimit() {
        return this.liftEncoderTwo.getPosition() <= -450;
    }

    public boolean threeAtLowerLimit() {
        return this.liftEncoderThree.getPosition() <= -450;
    }

    public boolean allAtLowerLimit() {
        return oneAtLowerLimit() && twoAtLowerLimit() && threeAtLowerLimit();
    }

    public void liftManual(double speed) {
        if (Math.abs(speed) > 0.05) {
            speed /= 2;
            // this.liftManualControl = true;
            moveLifter(speed);
        } else {
            // this.liftManualControl = false;
            moveLifter(0);
        }
    }

    public void raise() {
        // this.shouldbeLifted = true;
    }

    public void lower() {
        // this.shouldbeLifted = false;
    }

    public void load(double speed) {
        this.loader.set(speed);
    }

    public void raiseIntake() {
        // this.shouldbeLifted = true;
    }

    public void lowerIntake() {
        // this.shouldbeLifted = false;
    }

    public void setLoaderSpeed(double speed) {
        this.loader.set(speed);
    }

    public void manualIntake(double speed) {
        this.left.set(speed);
        this.front.set(speed);
        this.right.set(speed);
    }

    @Override
    public void periodic() {
        // runLifter();
        // runIntake();
    }

    private void monitorAndCorrectAlignment(double movementSpeed) {
        if (this.upLimitOne.isPressed() && !this.oneHasResetSinceSwitch) {
            this.liftEncoderOne.setPosition(0);
            this.oneHasResetSinceSwitch = true;
        } else if (!this.upLimitOne.isPressed() && this.oneHasResetSinceSwitch) {
            this.oneHasResetSinceSwitch = false;
        }

        if (this.upLimitTwo.isPressed() && !this.twoHasResetSinceSwitch) {
            this.liftEncoderTwo.setPosition(0);
            this.twoHasResetSinceSwitch = true;
        } else if (!this.upLimitTwo.isPressed() && this.twoHasResetSinceSwitch) {
            this.twoHasResetSinceSwitch = false;
        }

        if (this.upLimitThree.isPressed() && !this.threeHasResetSinceSwitch) {
            this.liftEncoderThree.setPosition(0);
            this.threeHasResetSinceSwitch = true;
        } else if (!this.upLimitTwo.isPressed() && this.threeHasResetSinceSwitch) {
            this.threeHasResetSinceSwitch = false;
        }
    }

    private void moveLifter(double speed) {
        monitorAndCorrectAlignment(speed);
        if (speed > 0 && allAtUpperLimit())
            speed = 0;
        else if (speed < 0 && allAtLowerLimit())
            speed = 0;

        this.lifterOne.set(this.oneAtUpperLimit() ? 0 : this.oneAtLowerLimit() ? 0 : speed);
        this.lifterTwo.set(this.twoAtUpperLimit() ? 0 : this.twoAtLowerLimit() ? 0 : speed);
        this.lifterThree.set(this.threeAtUpperLimit() ? 0 : this.threeAtLowerLimit() ? 0 : speed);
    }

    // private void runLifter() {
    // if (!liftManualControl) {
    // if (shouldbeLifted) {
    // } else {
    // }
    // }
    // }

    // private void runIntake() {
    // if (atUpperLimit()) {
    // this.left.set(1);
    // this.front.set(1);
    // this.right.set(1);

    // this.leftController.reset();
    // this.frontController.reset();
    // this.rightController.reset();
    // } else {
    // // TODO: logic for note movement while intake is out of the way
    // ChassisSpeeds velocities = this.driveSystem.getRobotVelocity();
    // double xSpeed = velocities.vxMetersPerSecond;
    // double ySpeed = velocities.vyMetersPerSecond;

    // // Untested!
    // this.left.set(this.leftController.calculate(getLeftSpeed(), xSpeed));
    // this.front.set(this.frontController.calculate(getFrontSpeed(), ySpeed));
    // this.right.set(this.rightController.calculate(getRightSpeed(), xSpeed));
    // }
    // }

    // private double getLeftSpeed() {
    // return rpmToMps(this.left.getEncoder().getVelocity());
    // }

    // private double getFrontSpeed() {
    // return rpmToMps(this.front.getEncoder().getVelocity());
    // }

    // private double getRightSpeed() {
    // return rpmToMps(this.right.getEncoder().getVelocity());
    // }

    // private double rpmToMps(double RPM) {
    // double RPS = RPM / 60;
    // double speedInMeterPerSecond = RPS *
    // IntakeConstants.intakeWheelCircunferenceMeters;
    // return speedInMeterPerSecond;
    // }

}
