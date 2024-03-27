package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;
    private final CANSparkMax aimMotor;
    private final SparkLimitSwitch aimUpperLimitSwitch;
    private final RelativeEncoder aimEncoder;
    private final PIDController aimPidController;

    private final double teleopMinAngle = 41.1; // Software limit
    private final double outputMin = 41; // Aim angle min
    private final double outputMax = 62; // Aim angle max
    private boolean hasZerodEncoder = false;

    public ShooterSubsystem() {
        this.topMotor = new CANSparkMax(ShooterConstants.topMotorId, MotorType.kBrushless);
        this.topMotor.setInverted(true);
        this.topMotor.setIdleMode(IdleMode.kBrake);
        this.topMotor.setSmartCurrentLimit(80);

        this.bottomMotor = new CANSparkMax(ShooterConstants.bottomMotorId, MotorType.kBrushless);
        this.bottomMotor.setInverted(true);
        this.bottomMotor.setIdleMode(IdleMode.kBrake);
        this.bottomMotor.setSmartCurrentLimit(80);

        this.aimMotor = new CANSparkMax(ShooterConstants.aimMotorId, MotorType.kBrushless);
        this.aimMotor.setInverted(false);
        this.aimMotor.setIdleMode(IdleMode.kBrake);
        this.topMotor.setSmartCurrentLimit(40);

        this.aimEncoder = this.aimMotor.getEncoder();
        this.aimPidController = new PIDController(ShooterConstants.aimControllerVals[0],
                ShooterConstants.aimControllerVals[1], ShooterConstants.aimControllerVals[2]);
        this.aimUpperLimitSwitch = this.aimMotor.getForwardLimitSwitch(Type.kNormallyOpen);

        ntInit();
    }

    private NetworkTable pidTuningPVs;
    private NetworkTableInstance table;
    private NetworkTableEntry angleOfShooter;
    private void ntInit() {
        table = NetworkTableInstance.getDefault();
        pidTuningPVs = table.getTable("pidTuningPVs");
        angleOfShooter = pidTuningPVs.getEntry("angleOfShooter");
    }
    private double holdTarget = 0;
    private boolean hasMoved = false;
    public void manualAim(double speed) {
        if(Math.abs(speed) > 0) {
            hasMoved = true;
            holdTarget = getAngle();
        }
        if(speed == 0 && hasMoved && hasZerodEncoder)
            autoAim(holdTarget);
        else
            moveAimMotor(speed);
    }

    public boolean autoAim() {
        return autoAim(49);
    }
    double maxAimSpeed = 0.2;
    double minAimSpeed = 0.03;
    public boolean autoAim(double targetAngle) {
        if(targetAngle == -1) {
            System.out.println("Warning: -1 is not a valid auto-aim value");
            return true;
        }
        if (!hasZerodEncoder) {
            moveAimMotor(0.4);
        } else {
            double aimSpeed = this.aimPidController.calculate(getAngle(), targetAngle);
            double giveOrTake = 0.3;
            if (aimSpeed < 0 && aimSpeed > -minAimSpeed) // Set minimum speed, otherwise it could get stuck at *just barely* off
                aimSpeed = -minAimSpeed;
            if (aimSpeed > 0 && aimSpeed < minAimSpeed) // Set minimum speed, otherwise it could get stuck at *just barely* off
                aimSpeed = minAimSpeed;
            if (Math.abs(targetAngle - getAngle()) <= giveOrTake) // Check if we're in margin of error
                aimSpeed = 0;
            moveAimMotor(Math.min(aimSpeed, maxAimSpeed));
            if(aimSpeed == 0) return true;
        }
        return false;
    }

    public void moveAimMotor(double speed) {
        if (hasZerodEncoder && speed <= 0 && getAngle() <= teleopMinAngle) {
            this.aimMotor.set(0);
            return;
        } 
        this.aimMotor.set(hasZerodEncoder ? speed : Math.max(speed, 0));
    }

    public double mapNormalizedToAngle(double normalized) {
        return (normalized * (outputMax - outputMin)) + outputMin;
    }

    public void spin(double mainSpeed) {
        this.topMotor.set(mainSpeed);
        this.bottomMotor.set(mainSpeed);
    }

    public void spinDifferently(double bottomSpeed, double topSpeed) {
        this.topMotor.set(topSpeed);
        this.bottomMotor.set(bottomSpeed);
    }

    public double getNeededAngleFromDistance(double distance) {
        double calculated = (-7.3595154 * distance) + 65.4586221; // When calibrating, replace this value with the generated equation
        return calculated > this.outputMax ? this.outputMax : calculated < this.outputMin ? this.outputMin : calculated; // TODO: move numbers to constants
    }

    double aimEncoderTop = 1.6223723883;

    public double getAngle() {
        double inputMin = 0; // min encoder position
        double inputMax = aimEncoderTop; // max encoder value
        double reading = this.aimEncoder.getPosition();
        double normalized = (reading - inputMin) / (inputMax - inputMin);
        return this.hasZerodEncoder ? mapNormalizedToAngle(normalized) : -1;
    }
    
    private double getRawPosition() {
        double reading = this.aimEncoder.getPosition();
        return reading;
    }

    @Override
    public void periodic() {
        if (!this.hasZerodEncoder) {
            if (this.aimUpperLimitSwitch.isPressed()) {
                this.hasZerodEncoder = true;
                this.aimEncoder.setPosition(aimEncoderTop);
            }
        }
        this.angleOfShooter.setDouble(this.getAngle());
    }

}
