package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake; // Assuming you have an intake/hopper subsystem
import frc.robot.subsystems.Shooter;
import java.util.function.DoubleSupplier;

public class FireFuel extends Command {
    private final Shooter m_shooter;
    private final Hood m_hood;
    private final Intake m_intake;
    
    // We pass the Limelight distance in as a supplier so it updates every loop!
    private final DoubleSupplier m_distanceSupplier;

    public FireFuel(Shooter shooter, Hood hood, Intake intake, DoubleSupplier distanceSupplier) {
        m_shooter = shooter;
        m_hood = hood;
        m_intake = intake;
        m_distanceSupplier = distanceSupplier;

        // The command needs to "own" these subsystems while it runs
        addRequirements(shooter, hood, intake);
    }

    @Override
    public void initialize() {
        // Optional: Turn on Limelight LEDs here if you keep them off by default
    }

    @Override
    public void execute() {
        // 1. Get the live distance from the Limelight
        double currentDistance = m_distanceSupplier.getAsDouble();

        // 2. Tell the Hood and Shooter to look at their maps and adjust!
        m_hood.setAngleFromDistance(currentDistance);
        m_shooter.setRPMFromDistance(currentDistance);

        // (If you have a Turret, you would also tell it to track the target here)

        // 3. THE PRE-FLIGHT CHECKLIST
        // We only spin the intake to feed the note/fuel IF both are ready.
        if (m_shooter.isAtSpeed() && m_hood.isAtPosition()) {
            // FIRE!
            m_intake.feedShooter(); 
        } else {
            // Hold the game piece steady while we wait for the motors to catch up
            m_intake.stop(); 
        }
    }

    @Override
    public boolean isFinished() {
        // We return false so this command runs for as long as the driver holds the button.
        // Once they let go, it is interrupted and the end() method is called.
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        // When the driver lets go of the trigger, shut everything down safely
        m_shooter.stop();
        m_hood.stop();
        m_intake.stop();
    }
}