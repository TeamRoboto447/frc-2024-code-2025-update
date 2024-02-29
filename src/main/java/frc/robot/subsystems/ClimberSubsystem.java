package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimberConstants;
import frc.robot.utils.EncoderBasedLimits;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax climbMotor;
    private final EncoderBasedLimits climbLimits;

    private boolean climberUp = false;

    public ClimberSubsystem() {
        this.climbMotor = new CANSparkMax(ClimberConstants.climbMotorId, MotorType.kBrushless);
        this.climbLimits = new EncoderBasedLimits(0, 1);
    }


    public void raise() {
        this.climberUp = true;
    }

    public void lower() {
        this.climberUp = false;
    }


    @Override
    public void periodic() {
        if (climberUp) {
            if (this.getClimberPos() >= this.climbLimits.getUpperLimit()) {
                this.climbMotor.set(1);
            } else {
                this.climbMotor.set(0);
            }
        } else {
            if (this.getClimberPos() <= this.climbLimits.getLowerLimit()) {
                this.climbMotor.set(-1);
            } else {
                this.climbMotor.set(0);
            }
        }
    }

    private double getClimberPos() {
        return this.climbMotor.getEncoder().getPosition();
    }
}
