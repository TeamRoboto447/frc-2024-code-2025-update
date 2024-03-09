package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

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

    private final CANSparkMax lifterOne;
    private final SparkLimitSwitch downLimitOne;
    private final SparkLimitSwitch upLimitOne;
    private final RelativeEncoder liftEncoderOne;
    private final CANSparkMax lifterTwo;
    private final SparkLimitSwitch downLimitTwo;
    private final SparkLimitSwitch upLimitTwo;
    private final RelativeEncoder liftEncoderTwo;
    private final CANSparkMax lifterThree;
    private final SparkLimitSwitch downLimitThree;
    private final SparkLimitSwitch upLimitThree;
    private final RelativeEncoder liftEncoderThree;

    private boolean shouldbeLifted = false;
    private boolean liftManualControl = false;

    private boolean correctingOne = false;
    private boolean correctingTwo = false;
    private boolean correctingThree = false;

    public IntakeSubsystem(SwerveSubsystem driveSystem) {
        this.driveSystem = driveSystem;

        this.left = new CANSparkMax(IntakeConstants.leftMotorId, MotorType.kBrushless);
        this.leftController = new PIDController(
                IntakeConstants.leftControllerVals[0],
                IntakeConstants.leftControllerVals[1],
                IntakeConstants.leftControllerVals[2]);
        this.left.setInverted(false);

        this.front = new CANSparkMax(IntakeConstants.frontMotorId, MotorType.kBrushless);
        this.frontController = new PIDController(
                IntakeConstants.frontControllerVals[0],
                IntakeConstants.frontControllerVals[1],
                IntakeConstants.frontControllerVals[2]);
        this.front.setInverted(false);

        this.right = new CANSparkMax(IntakeConstants.rightMotorId, MotorType.kBrushless);
        this.rightController = new PIDController(
                IntakeConstants.rightControllerVals[0],
                IntakeConstants.rightControllerVals[1],
                IntakeConstants.rightControllerVals[2]);
        this.right.setInverted(true);
        this.loader = new CANSparkMax(IntakeConstants.loaderMotorId, MotorType.kBrushless);

        this.lifterOne = new CANSparkMax(IntakeConstants.lifterOne, MotorType.kBrushless);
        this.lifterOne.setInverted(false);
        this.lifterOne.setIdleMode(IdleMode.kBrake);
        this.lifterTwo = new CANSparkMax(IntakeConstants.lifterTwo, MotorType.kBrushless);
        this.lifterTwo.setInverted(false);
        this.lifterOne.setIdleMode(IdleMode.kBrake);
        this.lifterThree = new CANSparkMax(IntakeConstants.lifterThree, MotorType.kBrushless);
        this.lifterThree.setInverted(false);
        this.lifterOne.setIdleMode(IdleMode.kBrake);

        this.upLimitOne = lifterOne.getForwardLimitSwitch(Type.kNormallyOpen);
        this.upLimitTwo = lifterTwo.getForwardLimitSwitch(Type.kNormallyOpen);
        this.upLimitThree = lifterThree.getForwardLimitSwitch(Type.kNormallyOpen);

        this.downLimitOne = lifterOne.getReverseLimitSwitch(Type.kNormallyOpen);
        this.downLimitTwo = lifterTwo.getReverseLimitSwitch(Type.kNormallyOpen);
        this.downLimitThree = lifterThree.getReverseLimitSwitch(Type.kNormallyOpen);

        this.liftEncoderOne = lifterOne.getEncoder();
        this.liftEncoderTwo = lifterTwo.getEncoder();
        this.liftEncoderThree = lifterThree.getEncoder();
    }

    public boolean atUpperLimit() {
        // return this.upLimitOne.isPressed() && this.upLimitTwo.isPressed() &&
        // this.upLimitThree.isPressed();
        return this.upLimitTwo.isPressed();
    }

    public boolean atLowerLimit() {
        // return this.downLimitOne.isPressed() && this.downLimitTwo.isPressed() &&
        // this.downLimitThree.isPressed();
        if(this.liftEncoderTwo.getPosition() <= -415) return true;
        return false;
    }

    public void liftManual(double speed) {
        if (Math.abs(speed) > 0.05) {
            speed /= 2;
            this.liftManualControl = true;
            moveLifter(speed);
        } else {
            this.liftManualControl = false;
            moveLifter(0);
        }
    }

    public void raise() {
        this.shouldbeLifted = true;
    }

    public void lower() {
        this.shouldbeLifted = false;
    }

    public void load(double speed) {
        this.loader.set(speed);
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

    public void manualIntake(double speed) {
        this.left.set(speed);
        this.front.set(speed/2);
        this.right.set(speed);
    }

    @Override
    public void periodic() {
        // runLifter();
        // runIntake();
    }

    // private boolean needsCorrection(double encoderPos, double avgPos, boolean positiveDir) {
    //     double allowedDeviation = 5;
    //     double deviation = encoderPos - avgPos;
    //     if(Math.abs(deviation) < allowedDeviation) return false;
    //     if(positiveDir && deviation > 0)
    //         System.out.println("Correcting because going up and deviation positive\nPos:"+encoderPos+" Avg:"+avgPos+" Dev: "+deviation);
    //     else if(!positiveDir && deviation < 0)
    //         return false;
    //     return false;
    // }

    private void monitorAndCorrectAlignment(double movementSpeed) {
        if (this.upLimitTwo.isPressed()) {
            this.liftEncoderOne.setPosition(0);
            this.liftEncoderTwo.setPosition(0);
            this.liftEncoderThree.setPosition(0);
        }
        
        // double encoderReadingSum = this.liftEncoderOne.getPosition() + this.liftEncoderTwo.getPosition()
        //         + this.liftEncoderThree.getPosition();
        // double avgEncoderReading = encoderReadingSum / 3;
        
        // if(needsCorrection(this.liftEncoderOne.getPosition(), avgEncoderReading, movementSpeed > 0)) {
        //     this.correctingOne = true;
        //     this.lifterOne.set(0);
        // } else
        // this.correctingOne = false;

        // if(needsCorrection(this.liftEncoderTwo.getPosition(), avgEncoderReading, movementSpeed > 0)) {
        //     this.correctingTwo = true;
        //     this.lifterTwo.set(0);
        // } else
        // this.correctingTwo = false;

        // if(needsCorrection(this.liftEncoderThree.getPosition(), avgEncoderReading, movementSpeed > 0)) {
        //     this.correctingThree = true;
        //     this.lifterThree.set(0);
        // } else
        // this.correctingThree = false;
    }

    private void moveLifter(double speed) {
        monitorAndCorrectAlignment(speed);
        if (speed > 0 && atUpperLimit())
            speed = 0;
        else if(speed < 0 && atLowerLimit())
            speed = 0;
        
        if (!this.correctingOne)
            this.lifterOne.set(speed);
        if (!this.correctingTwo)
            this.lifterTwo.set(speed);
        if (!this.correctingThree)
            this.lifterThree.set(speed);
    }

    private void runLifter() {
        if (!liftManualControl) {
            if (shouldbeLifted) {
            } else {
            }
        }
    }

    private void runIntake() {
        if (atUpperLimit()) {
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
