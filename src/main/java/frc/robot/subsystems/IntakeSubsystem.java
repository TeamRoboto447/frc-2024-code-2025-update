package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.EncoderBasedLimits;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax left;
    private final CANSparkMax front;
    private final CANSparkMax right;
    private final CANSparkMax loader;

    private final CANSparkMax lifterOne;
    private final EncoderBasedLimits lifterOneLimits;
    private final CANSparkMax lifterTwo;
    private final EncoderBasedLimits lifterTwoLimits;
    private final CANSparkMax lifterThree;
    private final EncoderBasedLimits lifterThreeLimits;

    private boolean shouldbeLifted = false;
    private boolean liftManualControl = false;

    public IntakeSubsystem() {
        this.left = new CANSparkMax(IntakeConstants.leftMotorId, MotorType.kBrushless);
        this.front = new CANSparkMax(IntakeConstants.frontMotorId, MotorType.kBrushless);
        this.right = new CANSparkMax(IntakeConstants.rightMotorId, MotorType.kBrushless);
        this.loader = new CANSparkMax(IntakeConstants.loaderMotorId, MotorType.kBrushless);

        this.lifterOne = new CANSparkMax(IntakeConstants.lifterOne, MotorType.kBrushless);
        this.lifterOne.setInverted(false);
        this.lifterTwo = new CANSparkMax(IntakeConstants.lifterTwo, MotorType.kBrushless);
        this.lifterTwo.setInverted(false);
        this.lifterThree = new CANSparkMax(IntakeConstants.lifterThree, MotorType.kBrushless);
        this.lifterThree.setInverted(false);

        this.lifterOneLimits = new EncoderBasedLimits(-1, 1);
        this.lifterTwoLimits = new EncoderBasedLimits(-1, 1);
        this.lifterThreeLimits = new EncoderBasedLimits(-1, 1);
    }

    public boolean atUpperLimit() {
        boolean lifterOneLimited = this.lifterOne.getEncoder().getPosition() >= this.lifterOneLimits.getUpperLimit();
        boolean lifterTwoLimited = this.lifterTwo.getEncoder().getPosition() >= this.lifterTwoLimits.getUpperLimit();
        boolean lifterThreeLimited = this.lifterThree.getEncoder().getPosition() >= this.lifterThreeLimits
                .getUpperLimit();
        return lifterOneLimited && lifterTwoLimited && lifterThreeLimited;
    }

    public boolean atLowerLimit() {
        boolean lifterOneLimited = this.lifterOne.getEncoder().getPosition() <= this.lifterOneLimits.getLowerLimit();
        boolean lifterTwoLimited = this.lifterTwo.getEncoder().getPosition() <= this.lifterTwoLimits.getLowerLimit();
        boolean lifterThreeLimited = this.lifterThree.getEncoder().getPosition() <= this.lifterThreeLimits
                .getLowerLimit();
        return lifterOneLimited && lifterTwoLimited && lifterThreeLimited;
    }

    public void liftManual(double speed) {
        if (Math.abs(speed) > 0.05) {
            speed /= 2;
            this.liftManualControl = true;
            this.lifterOne.set(speed);
            this.lifterTwo.set(speed);
            this.lifterThree.set(speed);
        } else {
            this.liftManualControl = false;
            this.lifterOne.set(0);
            this.lifterTwo.set(0);
            this.lifterThree.set(0);
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

    @Override
    public void periodic() {
        // runLifter();
        runIntake();
    }

    private void runLifter() {
        if (!liftManualControl) {
            if (shouldbeLifted) {
                if (!(this.lifterOne.getEncoder().getPosition() >= this.lifterOneLimits.getUpperLimit()))
                    lifterOne.set(1);
                else
                    lifterOne.set(0);

                if (!(this.lifterTwo.getEncoder().getPosition() >= this.lifterTwoLimits.getUpperLimit()))
                    lifterTwo.set(1);
                else
                    lifterTwo.set(0);

                if (!(this.lifterThree.getEncoder().getPosition() >= this.lifterThreeLimits.getUpperLimit()))
                    lifterThree.set(1);
                else
                    lifterThree.set(0);
            } else {
                if (!(this.lifterOne.getEncoder().getPosition() <= this.lifterOneLimits.getLowerLimit()))
                    lifterOne.set(-1);
                else
                    lifterOne.set(0);

                if (!(this.lifterTwo.getEncoder().getPosition() <= this.lifterTwoLimits.getLowerLimit()))
                    lifterTwo.set(-1);
                else
                    lifterTwo.set(0);

                if (!(this.lifterThree.getEncoder().getPosition() <= this.lifterThreeLimits.getLowerLimit()))
                    lifterThree.set(-1);
                else
                    lifterThree.set(0);
            }
        }
    }

    private void runIntake() {
        if (atUpperLimit()) {
            this.left.set(1);
            this.front.set(1);
            this.right.set(1);
        } else {
            // TODO: logic for note movement while intake is out of the way
        }
    }

}
