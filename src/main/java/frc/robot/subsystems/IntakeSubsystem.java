package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public enum IntakeStatus {
        LOWERED,
        IDLE,
        RAISED
    }

    // private final SwerveSubsystem driveSystem;
    private final PIDController liftCtrlOne = new PIDController(0.05, 0, 0);
    private final PIDController liftCtrlTwo = new PIDController(0.05, 0, 0);
    private final PIDController liftCtrlThree = new PIDController(0.05, 0, 0);

    private final CANSparkMax left;
    private final CANSparkMax front;
    private final CANSparkMax right;
    private final CANSparkMax loader;

    private final CANSparkMax lifterOne;
    private final SparkLimitSwitch upLimitLeft;
    private final RelativeEncoder liftEncoderLeft;
    private final CANSparkMax lifterTwo;
    private final SparkLimitSwitch upLimitMid;
    private final RelativeEncoder leftEncoderMid;
    private final CANSparkMax lifterThree;
    private final SparkLimitSwitch upLimitThree;
    private final RelativeEncoder liftEncoderRight;

    private IntakeStatus requestedLiftStatus = IntakeStatus.RAISED;
    private boolean liftManualControl = true;

    private boolean leftHasResetSinceSwitch = false;
    private boolean midHasResetSinceSwitch = false;
    private boolean rightHasResetSinceSwitch = false;

    private final double liftMin = -450;
    private boolean zerodMotors = false;
    private boolean zerodLeftLift = false;
    private boolean zerodMidLift = false;
    private boolean zerodRightLift = false;

    public IntakeSubsystem(SwerveSubsystem driveSystem) {
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
        this.lifterOne.setSmartCurrentLimit(20);
        this.setPeriods(this.lifterOne);

        this.lifterTwo = new CANSparkMax(IntakeConstants.lifterTwo, MotorType.kBrushless);
        this.lifterTwo.setInverted(false);
        this.lifterTwo.setIdleMode(IdleMode.kBrake);
        this.lifterTwo.setSmartCurrentLimit(20);
        this.setPeriods(this.lifterTwo);

        this.lifterThree = new CANSparkMax(IntakeConstants.lifterThree, MotorType.kBrushless);
        this.lifterThree.setInverted(false);
        this.lifterOne.setIdleMode(IdleMode.kBrake);
        this.lifterThree.setSmartCurrentLimit(20);
        this.setPeriods(this.lifterThree);

        this.upLimitLeft = lifterOne.getForwardLimitSwitch(Type.kNormallyOpen);
        this.upLimitMid = lifterTwo.getForwardLimitSwitch(Type.kNormallyOpen);
        this.upLimitThree = lifterThree.getForwardLimitSwitch(Type.kNormallyOpen);

        // this.downLimitOne = lifterOne.getReverseLimitSwitch(Type.kNormallyOpen);
        // this.downLimitTwo = lifterTwo.getReverseLimitSwitch(Type.kNormallyOpen);
        // this.downLimitThree = lifterThree.getReverseLimitSwitch(Type.kNormallyOpen);

        this.liftEncoderLeft = lifterOne.getEncoder();
        this.leftEncoderMid = lifterTwo.getEncoder();
        this.liftEncoderRight = lifterThree.getEncoder();

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
        return this.upLimitLeft.isPressed();
    }

    public boolean twoAtUpperLimit() {
        return this.upLimitMid.isPressed();
    }

    public boolean threeAtUpperLimit() {
        return this.upLimitThree.isPressed();
    }

    public boolean allAtUpperLimit() {
        return oneAtUpperLimit() && twoAtUpperLimit() && threeAtUpperLimit();
    }

    public boolean oneAtLowerLimit() {
        return this.liftEncoderLeft.getPosition() <= this.liftMin;
    }

    public boolean twoAtLowerLimit() {
        return this.leftEncoderMid.getPosition() <= this.liftMin;
    }

    public boolean threeAtLowerLimit() {
        return this.liftEncoderRight.getPosition() <= this.liftMin;
    }

    public boolean allAtLowerLimit() {
        return oneAtLowerLimit() && twoAtLowerLimit() && threeAtLowerLimit();
    }

    public void liftManual(double speed) {
        if (Math.abs(speed) > 0.05) {
            this.liftManualControl = true;
            moveLifter(speed);
        } else {
            // this.liftManualControl = false;
            // this.liftCtrlOne.reset();
            // this.liftCtrlTwo.reset();
            // this.liftCtrlThree.reset();
            moveLifter(0);
        }
    }

    public void load(double speed) {
        this.loader.set(speed);
    }

    public void raiseIntake() {
        this.requestedLiftStatus = IntakeStatus.RAISED;
    }

    public void idleIntake() {
        this.requestedLiftStatus = IntakeStatus.IDLE;
    }

    public void lowerIntake() {
        this.requestedLiftStatus = IntakeStatus.LOWERED;
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
        runLifter();
        // runIntake();
    }

    private void monitorAndCorrectAlignment(double movementSpeed) {
        if (this.zerodMotors)
            return;
        if (this.upLimitMid.isPressed() && !this.leftHasResetSinceSwitch) {
            this.liftEncoderLeft.setPosition(0);
            this.zerodLeftLift = true;
            this.leftHasResetSinceSwitch = true;
        } else if (!this.upLimitMid.isPressed() && this.leftHasResetSinceSwitch) {
            this.leftHasResetSinceSwitch = false;
        }

        if (this.upLimitMid.isPressed() && !this.midHasResetSinceSwitch) {
            this.leftEncoderMid.setPosition(0);
            this.zerodMidLift = true;
            this.midHasResetSinceSwitch = true;
        } else if (!this.upLimitMid.isPressed() && this.midHasResetSinceSwitch) {
            this.midHasResetSinceSwitch = false;
        }

        if (this.upLimitMid.isPressed() && !this.rightHasResetSinceSwitch) {
            this.liftEncoderRight.setPosition(0);
            this.zerodRightLift = true;
            this.rightHasResetSinceSwitch = true;
        } else if (!this.upLimitMid.isPressed() && this.rightHasResetSinceSwitch) {
            this.rightHasResetSinceSwitch = false;
        }

        this.zerodMotors = this.zerodLeftLift && this.zerodMidLift && zerodRightLift;
    }

    private void moveLifter(double speed) {
        monitorAndCorrectAlignment(speed);
        if (speed > 0 && oneAtUpperLimit())
            speed = 0;
        if (speed < 0 && oneAtLowerLimit())
            speed = 0;
        moveLifterOne(speed);
        moveLifterTwo(speed);
        moveLifterThree(speed);
    }

    private void moveLifterOne(double speed) {
        if (speed < 0 && !this.zerodLeftLift)
            speed = 0;
        this.lifterOne.set(this.oneAtLowerLimit() && speed < 0 ? 0 : speed);
    }

    private void moveLifterTwo(double speed) {
        if (speed < 0 && !this.zerodMidLift)
            speed = 0;
        this.lifterTwo.set(this.twoAtLowerLimit() && speed < 0 ? 0 : speed);
    }

    private void moveLifterThree(double speed) {
        if (speed < 0 && !this.zerodRightLift)
            speed = 0;
        this.lifterThree.set(this.threeAtLowerLimit() && speed < 0 ? 0 : speed);
    }

    // private double getSpeed(double reading, double setpoint, double margin, double maxSpeed) {
    //     if (Math.abs(setpoint - reading) <= margin)
    //         return 0;
    //     if (reading < setpoint)
    //         return maxSpeed;
    //     if (reading > setpoint)
    //         return -maxSpeed;
    //     return 0;
    // };

    private void runLifter() {
        double setpoint = 0;
        if (!liftManualControl) {
            switch (requestedLiftStatus) {
                case RAISED:
                    setpoint = 0;
                    break;

                case IDLE:
                    setpoint = this.liftMin / 2;
                    break;

                case LOWERED:
                    setpoint = this.liftMin;
                    break;
            }
            double maxSpeed = 0.125;
            if (this.zerodMotors) {
                double measuredOne = this.liftEncoderLeft.getPosition(); // Control Master, slave measurements will include error from master to keep in sync
                double measuredTwo = this.leftEncoderMid.getPosition(); // Slave 1
                measuredTwo += (measuredTwo - measuredOne); // Add error to measurement to keep them more in sync
                double measuredThree = this.liftEncoderRight.getPosition(); // Slave 2
                measuredThree += (measuredThree - measuredOne); // Add error to measurement to keep them more in sync   
                // moveLifterOne(getSpeed(measuredOne, setpoint, 10, maxSpeed));
                // moveLifterTwo(getSpeed(measuredTwo, setpoint, 10, maxSpeed));
                // moveLifterThree(getSpeed(measuredThree, setpoint, 10, maxSpeed));
                moveLifterOne(this.liftCtrlOne.calculate(measuredOne, setpoint));
                moveLifterTwo(this.liftCtrlTwo.calculate(measuredTwo, setpoint));
                moveLifterThree(this.liftCtrlThree.calculate(measuredThree, setpoint));
                SmartDashboard.putNumberArray("Lift: Measured, Setpoint",
                        new double[] { measuredOne, measuredTwo, measuredThree, setpoint });
            } else {
                moveLifter(maxSpeed);
            }
        }
    }

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
