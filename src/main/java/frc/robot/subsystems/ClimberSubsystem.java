package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ClimberConstants;
import frc.robot.utils.EncoderBasedLimits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climbMotor;
    private final EncoderBasedLimits climbLimits;

    private boolean climberUp = false;

    public ClimberSubsystem() {
        this.climbMotor = new TalonFX(ClimberConstants.climbMotorId);
        this.climbLimits = new EncoderBasedLimits(0, 1);
    }

    public void raise() {
        this.climberUp = true;
    }

    public void lower() {
        this.climberUp = false;
    }

    public void manualClimb(double speed) {
        this.climbMotor.set(speed);
    }

    @Override
    public void periodic() {
        // if (climberUp) {
        // if (this.getClimberPos() >= this.climbLimits.getUpperLimit()) {
        // this.climbMotor.set(1);
        // } else {
        // this.climbMotor.set(0);
        // }
        // } else {
        // if (this.getClimberPos() <= this.climbLimits.getLowerLimit()) {
        // this.climbMotor.set(-1);
        // } else {
        // this.climbMotor.set(0);
        // }
        // }
    }

    private Angle getClimberPos() {
        return this.climbMotor.getRotorPosition().getValue();
    }
}
