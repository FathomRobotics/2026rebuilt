package frc.robot.subsystems.Encoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class ThroughBoreEncoder extends SubsystemBase {

    private Encoder encoder;
    private Rotation2d m_offset = Rotation2d.fromDegrees(0);
    private boolean m_Inverted;

    public ThroughBoreEncoder(int dioChannel){
        encoder = new Encoder(dioChannel, 1);
        encoder.setDistancePerPulse(1.0/8192.0);
    }

    public double getDistance() {
        return (encoder.getDistance());
    }
    public void doReset() {
        encoder.reset();
    }
}