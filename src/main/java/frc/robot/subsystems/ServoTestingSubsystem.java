package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoTestingSubsystem extends SubsystemBase{
    private final Servo servo0;
    private final Servo servo1;
    public ServoTestingSubsystem() {
        servo0 = new Servo(0);
        servo1 = new Servo(1);
    }

    public void setPosition(double position) {
        servo0.setPosition(position);
        servo1.setPosition(position);
    }
}
