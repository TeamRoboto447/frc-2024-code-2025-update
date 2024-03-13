package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax aimMotor;
    private final SparkLimitSwitch aimUpperLimitSwitch;
    private final RelativeEncoder aimEncoder;
    private final PIDController aimPidController;

    private final double teleopMinAngle = 40.25; // Software limit
    private final double outputMin = 40; // Aim angle min
    private final double outputMax = 58; // Aim angle max
    private boolean hasZerodEncoder = false;

    public ShooterSubsystem() {
        this.leftMotor = new CANSparkMax(ShooterConstants.leftMotorId, MotorType.kBrushless);
        this.leftMotor.setInverted(false);
        this.leftMotor.setIdleMode(IdleMode.kBrake);

        this.rightMotor = new CANSparkMax(ShooterConstants.rightMotorId, MotorType.kBrushless);
        this.rightMotor.setInverted(true);
        this.rightMotor.setIdleMode(IdleMode.kBrake);

        this.aimMotor = new CANSparkMax(ShooterConstants.aimMotorId, MotorType.kBrushless);
        this.aimMotor.setInverted(false);
        this.aimMotor.setIdleMode(IdleMode.kBrake);

        this.aimEncoder = this.aimMotor.getEncoder();
        this.aimPidController = new PIDController(ShooterConstants.aimControllerVals[0],
                ShooterConstants.aimControllerVals[1], ShooterConstants.aimControllerVals[2]);
        this.aimUpperLimitSwitch = this.aimMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    }

    public void manualAim(double speed) {
        moveAimMotor(speed);
    }

    public void autoAim() {
        autoAim(49);
    }

    public void autoAim(double targetAngle) {
        if (!hasZerodEncoder) {
            moveAimMotor(1);
        } else {
            double aimSpeed = this.aimPidController.calculate(getAngle(), targetAngle);
            double giveOrTake = 0.5;
            if (Math.abs(targetAngle - getAngle()) <= giveOrTake)
                aimSpeed = 0;
            moveAimMotor(Math.min(aimSpeed, 0.4));

        }
    }

    public void moveAimMotor(double speed) {
        if (hasZerodEncoder && speed <= 0 && getAngle() <= teleopMinAngle) {
            System.out.println(getAngle());
            this.aimMotor.set(0);
            return;
        }
        this.aimMotor.set(hasZerodEncoder ? speed : Math.max(speed * 3, 0));
    }

    public double mapNormalizedToAngle(double normalized) {
        return (normalized * (outputMax - outputMin)) + outputMin;
    }

    public void spin(double mainSpeed) {
        // int sign = mainSpeed >= 0 ? 1 : -1;
        // double rightSpeed = (Math.min(Math.abs(mainSpeed), 0.05)) * sign;
        this.leftMotor.set(mainSpeed);
        // this.rightMotor.set(rightSpeed);
        this.rightMotor.set(mainSpeed);
    }

    private double getAngle() {
        double inputMin = 0; // min encoder position
        double inputMax = 1.8092646; // max encoder value
        double reading = this.aimEncoder.getPosition();
        double normalized = (reading - inputMin) / (inputMax - inputMin);
        return mapNormalizedToAngle(normalized);
    }

    @Override
    public void periodic() {
        if (!this.hasZerodEncoder) {
            if (this.aimUpperLimitSwitch.isPressed()) {
                this.hasZerodEncoder = true;
                this.aimEncoder.setPosition(1.8092646);
            }
        }
    }

}
