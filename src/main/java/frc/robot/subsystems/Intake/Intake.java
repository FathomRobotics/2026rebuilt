package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    // Initialize motors; references a separate config file (IntakeConfig.java) for the device ID.
    // I don't know what the CANBus is.
    private TalonFX leftIntakeMotor = new TalonFX(IntakeConfig.leftIntakeMotorID, CANBus.roboRIO());
    private TalonFX rightIntakeMotor = new TalonFX(IntakeConfig.rightIntakeMotorID, CANBus.roboRIO());

    // I honestly don't know. I'm just basing this off of Jake's code.
    public Intake(){

    }

    // Sets the power of the motors to a given value.
    public void setIntakePower(double value){
        leftIntakeMotor.set(value);
        rightIntakeMotor.set(-value);
    }

    // Runs the setIntakePower function from a command that can be used in other files?
    public Command setIntakePowerCommand(double value){
        return runOnce( () -> setIntakePower(value));
    }

}
