package frc.robot.subsystems;

public class states {
    public enum shooterState {
        IDLE,
        READY_TO_SHOOT, 
        SHOOTING
    }

    public enum intakeState {
        IDLE(0), 
        INTAKING(0.58), 
        EXTENDING(0.58), 
        EXTENDED(0.58), 
        //0.344
        RETRACTING(0),
        //0.445
        AGITATE(0.58);
        private double intakePose;

        intakeState(double intakePose){
            this.intakePose = intakePose;
        }
        public double getIntakePose(){
            return this.intakePose;
        }

    }

    public enum spindexState {
        IDLE, 
        RUNNING
    }

    public enum turretspinState {
        IDLE, 
        TRACKING
    }
    
    public enum hoodState {
        DOWN(0), 
        UP(1.5);
        private double hoodPose;

        hoodState(double hoodPose){
            this.hoodPose = hoodPose;
        }
        public double getHoodPose(){
            return this.hoodPose;
        }

    }
}
