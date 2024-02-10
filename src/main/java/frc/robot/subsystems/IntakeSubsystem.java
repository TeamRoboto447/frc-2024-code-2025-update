package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax left;
    private final CANSparkMax front;
    private final CANSparkMax right;
    private final CANSparkMax loader;
    private final Servo platformLifter0;
    private final Servo platformLifter1;

    private boolean shouldbeLifted = false;

    public IntakeSubsystem() {
        this.left = new CANSparkMax(IntakeConstants.leftMotorId, MotorType.kBrushless);
        this.front = new CANSparkMax(IntakeConstants.frontMotorId, MotorType.kBrushless);
        this.right = new CANSparkMax(IntakeConstants.rightMotorId, MotorType.kBrushless);
        this.loader = new CANSparkMax(IntakeConstants.loaderMotorId, MotorType.kBrushless);

        this.platformLifter0 = new Servo(IntakeConstants.lifter0Channel);
        this.platformLifter1 = new Servo(IntakeConstants.lifter1Channel);
    }

    public boolean isIntakeUp() {
        return this.shouldbeLifted; // TODO: Detect if lifted rather than just trusting the variable
    }

    @Override
    public void periodic() {
        runServos();
        runIntake();
    }

    private void runServos() {
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
        } else {
            // TODO: logic for note movement while intake is out of the way
        }
    }

}
