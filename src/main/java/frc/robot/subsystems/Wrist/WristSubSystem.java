package frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase{
    private WristIO m_io;
    private WristIOInputsAutoLogged m_input;


    public WristSubsystem(WristIO io){
        m_io = io;

        m_input = new WristIOInputsAutoLogged();
    }

    //setters
    public void setWristAngle(double angle) {
        m_io.setWristAngle(angle);
    }

    public void setWristPower(double speed) {
        m_io.setWristAngle(speed);
    }

    public CommandBase setWristPowerFactory(double speed) {
        return run(
            () -> {
                setWristPower(speed);
        });
    }

    public void setIntakeSpeed(double speed) {
        m_io.setIntakeSpeed(speed);
    }

    public CommandBase setIntakeSpeedFactory(double speed) {
        return run(
            () -> {
                setIntakeSpeed(speed);
        });
    }


    //getters
    public double getWristAngle() {
        return m_io.getWristAngle();
    }

    public double getIntakeAmps() {
        return m_io.getIntakeAmps();
    }


    public void periodic() {
        m_io.updateInputs(m_input);
    }

    public boolean atLimit() {
        return m_io.atLimit();
    }
}
