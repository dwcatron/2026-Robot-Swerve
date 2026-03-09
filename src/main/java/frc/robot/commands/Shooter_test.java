package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shooter_test extends Command {
    private final Shooter m_shooter;
    private final double m_targetRPM = 4500;

    public Shooter_test(Shooter shooter) {
        m_shooter = shooter;
        
        // This is required so the Command Scheduler knows this command controls the shooter
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        // You can put setRPM here or in execute. 
        // Putting it here is slightly more efficient if the RPM doesn't change.
        m_shooter.setRPM(m_targetRPM);
    }

    @Override
    public void execute() {
        // We don't need to repeatedly send the RPM request if it's constant, 
        // Phoenix 6 closed-loop handles it in the background!
    }

    @Override
    public boolean isFinished() {
        // Return false so the command keeps running as long as the button is held
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // The exact moment the driver lets go of the button, stop the motors
        m_shooter.stop();
    }
}