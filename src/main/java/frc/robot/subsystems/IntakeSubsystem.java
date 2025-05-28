package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    public enum IntakeStatus {
        LOWERED,
        IDLE,
        RAISED,
        MOVING,
        STALLED_AT_BOTTOM
    }

    private final SparkMax left;
    private final SparkMax front;
    private final SparkMax right;
    private final SparkMax loader;

    private final SparkMax lifterLeft;
    private final RelativeEncoder liftEncoderLeft;
    private final SparkMax lifterMid;
    private final SparkLimitSwitch upLimitMid;
    private final RelativeEncoder liftEncoderMid;
    private final SparkMax lifterRight;
    private final RelativeEncoder liftEncoderRight;

    private IntakeStatus requestedLiftStatus = IntakeStatus.RAISED;
    private boolean liftManualControl = false;

    private final double liftMin = -460.2;
    private boolean cantMoveLower = false;
    private boolean zerodMotors = false;

    public IntakeSubsystem(SwerveSubsystem driveSystem) {
        this.left = new SparkMax(IntakeConstants.leftMotorId, MotorType.kBrushless);
        this.left.setInverted(false);
        this.setPeriods(this.left);

        this.front = new SparkMax(IntakeConstants.frontMotorId, MotorType.kBrushless);
        this.front.setInverted(false);
        this.setPeriods(this.front);

        this.right = new SparkMax(IntakeConstants.rightMotorId, MotorType.kBrushless);
        this.right.setInverted(true);
        this.setPeriods(this.right);

        this.loader = new SparkMax(IntakeConstants.loaderMotorId, MotorType.kBrushless);

        this.lifterMid = new SparkMax(IntakeConstants.lifterTwo, MotorType.kBrushless);
        this.lifterMid.setInverted(false);
        this.lifterMid.setIdleMode(IdleMode.kBrake);
        this.lifterMid.setSmartCurrentLimit(20);
        this.setPeriods(this.lifterMid);

        this.lifterLeft = new SparkMax(IntakeConstants.lifterOne, MotorType.kBrushless);
        this.lifterLeft.setInverted(false);
        this.lifterLeft.setIdleMode(IdleMode.kBrake);
        this.lifterLeft.setSmartCurrentLimit(20);
        this.setPeriods(this.lifterLeft);

        this.lifterRight = new SparkMax(IntakeConstants.lifterThree, MotorType.kBrushless);
        this.lifterRight.setInverted(false);
        this.lifterRight.follow(this.lifterMid);
        this.lifterRight.setSmartCurrentLimit(20);
        this.setPeriods(this.lifterRight);

        this.upLimitMid = lifterMid.getForwardLimitSwitch();

        this.liftEncoderLeft = lifterLeft.getEncoder();
        this.liftEncoderMid = lifterMid.getEncoder();
        this.liftEncoderRight = lifterRight.getEncoder();

    }

    private void setPeriods(SparkMax sparkMax) {
        sparkMax.setCANTimeout(500);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);
    }

    public boolean atUpperLimit() {
        return getState() == IntakeStatus.RAISED;
    }

    public boolean atLowerLimit() {
        return getState() == IntakeStatus.LOWERED || getState() == IntakeStatus.STALLED_AT_BOTTOM;
    }

    public void liftManual(double speed) {
        if (Math.abs(speed) > 0.05) {
            this.liftManualControl = true;
            moveLifter(speed);
        } else {
            // this.liftManualControl = false;
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
        // if (getState() != IntakeStatus.LOWERED && getState() !=
        // IntakeStatus.STALLED_AT_BOTTOM)
        // speed = Math.min(speed, 0);
        this.left.set(speed);
        this.front.set(speed);
        this.right.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Lift: Requested State", this.requestedLiftStatus.name());
        SmartDashboard.putString("Lift: Current State", getState().name());
        runLifter();
    }

    private double leftPrevPos = Double.MAX_VALUE;
    private double midPrevPos = Double.MAX_VALUE;
    private double rightPrevPos = Double.MAX_VALUE;

    public void resetLowerLimitMonitor() {
        leftPrevPos = Double.MAX_VALUE;
        midPrevPos = Double.MAX_VALUE;
        rightPrevPos = Double.MAX_VALUE;
        cantMoveLower = false;
    }

    private void monitorLimits(double movementSpeed) {
        if (this.upLimitMid.isPressed()) {
            this.liftEncoderLeft.setPosition(0);
            this.liftEncoderMid.setPosition(0);
            this.liftEncoderRight.setPosition(0);
            this.zerodMotors = true;
        }
        double leftPos = this.liftEncoderLeft.getPosition();
        double midPos = this.liftEncoderMid.getPosition();
        double rightPos = this.liftEncoderRight.getPosition();
        boolean leftMoving = Math.abs(leftPos - this.leftPrevPos) > 1;
        this.leftPrevPos = leftPos;
        boolean midMoving = Math.abs(midPos - this.midPrevPos) > 1;
        this.midPrevPos = midPos;
        boolean rightMoving = Math.abs(rightPos - this.rightPrevPos) > 1;
        this.rightPrevPos = rightPos;
        boolean moving = leftMoving || midMoving || rightMoving;

        if (moving == false && movementSpeed < 0 && midPos < -300)
            cantMoveLower = true;
        else if (moving == true && movementSpeed < 0)
            cantMoveLower = false;
        else if (movementSpeed >= 0)
            resetLowerLimitMonitor();
    }

    private void moveLifter(double speed) {
        this.lifterLeft.set(speed);
        this.lifterRight.set(speed);
        monitorLimits(speed);
        if (speed > 0 && atUpperLimit())
            speed = 0;
        if (speed < 0 && atLowerLimit())
            speed = 0;
        this.lifterMid.set(speed);
    }

    private double liftPosMargin = 1;

    public IntakeStatus getState() {
        double pos = this.liftEncoderMid.getPosition();
        if (this.upLimitMid.isPressed())
            return IntakeStatus.RAISED;
        if (Math.abs(this.liftEncoderMid.getPosition() - (this.liftMin / 2)) < liftPosMargin)
            return IntakeStatus.IDLE;
        if (pos <= this.liftMin)
            return IntakeStatus.LOWERED;
        if (cantMoveLower)
            return IntakeStatus.STALLED_AT_BOTTOM;
        return IntakeStatus.MOVING;
    }

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

                case MOVING:
                    System.err.println("Requested intake state should never be 'MOVING'");
                    break;

                case STALLED_AT_BOTTOM:
                    System.err.println("Requested intake state should never be 'STALLED_AT_BOTTOM'");
                    break;
            }
            double maxSpeed = 0.125;
            if (this.zerodMotors) {
                double pos = this.liftEncoderMid.getPosition();
                boolean atTarget = false;
                double error = Math.abs(this.liftEncoderMid.getPosition() - setpoint);
                if (setpoint == 0)
                    atTarget = getState() == IntakeStatus.RAISED;
                if (setpoint == this.liftMin)
                    atTarget = getState() == IntakeStatus.LOWERED;
                else // This setpoint may not directly match the IDLE position in getState(), so we
                     // calculate this ourselves
                    atTarget = (error < liftPosMargin);
                double speed = atTarget ? 0 : (setpoint > pos || setpoint == 0) ? maxSpeed : -maxSpeed;
                speed = error < 10 ? speed * 0.25 : speed; // Slow it down if close
                moveLifter(speed);
            } else {
                moveLifter(maxSpeed);
            }
        }
    }
}
