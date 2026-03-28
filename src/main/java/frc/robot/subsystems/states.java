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
        INTAKING(3), 
        EXTENDING(3), 
        EXTENDED(3), 
        RETRACTING(0),
        AGITATE(5);
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
