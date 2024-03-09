package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax aimMotor;

    private final AnalogInput aimPotentiometer;
    private final PIDController aimPidController;
    
    private final double teleopMinAngle = 45; // Software limit
    private final double outputMin = 40; // Aim angle min
    private final double outputMax = 58; // Aim angle max

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

        this.aimPotentiometer = new AnalogInput(ShooterConstants.aimPotentiometer);
        this.aimPidController = new PIDController(ShooterConstants.aimControllerVals[0], ShooterConstants.aimControllerVals[1], ShooterConstants.aimControllerVals[2]);
    }

    public void manualAim(double speed) {
        moveAimMotor(speed);
    }

    public void autoAim() {
        autoAim(49);
    }

    public void autoAim(double targetAngle) {
        double aimSpeed = this.aimPidController.calculate(getAngle(), targetAngle);
        double giveOrTake = 0.5;
        if(Math.abs(targetAngle - getAngle()) <= giveOrTake)
            aimSpeed = 0;
        moveAimMotor(aimSpeed);
    }

    public void moveAimMotor(double speed) {
        if(speed <= 0 && getAngle() <= teleopMinAngle) {
            this.aimMotor.set(0);
            return;
        }
        this.aimMotor.set(speed);
    }

    public double mapNormalizedToAngle(double normalized) {
        return (normalized * (outputMax - outputMin)) + outputMin;
    }

    public void spin(double mainSpeed) {
        int sign = mainSpeed >= 0 ? 1 : -1;
        // double rightSpeed = (Math.min(Math.abs(mainSpeed), 0.05)) * sign;
        this.leftMotor.set(mainSpeed);
        // this.rightMotor.set(rightSpeed);
        this.rightMotor.set(mainSpeed);
    }

    private double getAngle() {
        double inputMin = 1.02197163; // Pot voltage min
        double inputMax = 1.65039045; // Pot Voltage max
        double reading = this.aimPotentiometer.getVoltage();
        double normalized = (reading - inputMin) / (inputMax - inputMin);
        return mapNormalizedToAngle(normalized);
    }

    @Override
    public void periodic() {
        // System.out.println(this.aimPotentiometer.getVoltage());
    }

}
