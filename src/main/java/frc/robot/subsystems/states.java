package frc.robot.subsystems;

public class states {
    public enum shooterState {
        IDLE, 
        SPINNING_UP, 
        READY_TO_SHOOT, 
        SHOOTING
    }

    public enum intakeState {
        IDLE(-1), 
        INTAKING(3), 
        EXTENDING(-1.35), //-2.9
        EXTENDED(2), 
        RETRACTING(-0.6),
        AGITATE(2);
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
