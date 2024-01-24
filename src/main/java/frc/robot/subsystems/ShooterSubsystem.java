package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    public ShooterSubsystem() {
        this.leftMotor = new CANSparkMax(Constants.ShooterConstants.leftMotorId, MotorType.kBrushless);
        this.leftMotor.setInverted(false);
        this.leftMotor.setIdleMode(IdleMode.kBrake);

        this.rightMotor = new CANSparkMax(Constants.ShooterConstants.rightMotorId, MotorType.kBrushless);
        this.rightMotor.follow(this.leftMotor, true);
        this.rightMotor.setIdleMode(IdleMode.kBrake);
    }

    public void spin(double speed) {
        this.leftMotor.set(speed);
        SmartDashboard.putNumber("Shooter Speed", speed);
    }

}
