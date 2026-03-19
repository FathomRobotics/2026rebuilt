package frc.robot.subsystems;

public class states {
    public enum shooterState {
        IDLE, 
        SPINNING_UP, 
        READY_TO_SHOOT, 
        SHOOTING
    }

    public enum intakeState {
        IDLE(0), 
        INTAKING(15), 
        EXTENDING(15), 
        EXTENDED(15), 
        RETRACTING(0);
        
        private double intakePose;

        intakeState(double intakePose) {
            this.intakePose = intakePose;
        }

        public double getIntakePose() {
            return this.intakePose;
        }
    }

    public enum spindexState {
        IDLE, 
        RUNNING
    }

}
