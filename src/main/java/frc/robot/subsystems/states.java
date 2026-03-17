package frc.robot.subsystems;

public class states {
    public enum shooterState {
        IDLE, 
        SPINNING_UP, 
        READY_TO_SHOOT, 
        SHOOTING
    }

    public enum intakeState {
        IDLE, 
        INTAKING, 
        EXTENDING, 
        EXTENDED, 
        RETRACTING
    }

    public enum spindexState {
        IDLE, 
        RUNNING
    }
}
