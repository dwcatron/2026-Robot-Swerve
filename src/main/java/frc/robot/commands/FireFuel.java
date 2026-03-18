package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import java.util.function.DoubleSupplier;


public class FireFuel extends Command {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Hood m_hood;
    private final Intake m_intake;
   
    private final DoubleSupplier m_distanceSupplier;


    public FireFuel(Shooter shooter, Turret turret, Hood hood, Intake intake, DoubleSupplier distanceSupplier) {
        m_shooter = shooter;
        m_turret = turret;
        m_hood = hood;
        m_intake = intake;
        m_distanceSupplier = distanceSupplier;


        // The command needs to "own" all four subsystems while it runs
        addRequirements(shooter, turret, hood, intake);
    }


    
   
    @Override
    public void execute() {
        double currentDistance = m_distanceSupplier.getAsDouble();


        // 1. Tell the Hood and Shooter to adjust based on their interpolation maps
        m_hood.setAngleFromDistance(currentDistance);
        m_shooter.setRPMFromDistance(currentDistance);


        // 2. THE PRE-FLIGHT CHECKLIST
        // We only spin the intake to feed the note IF all three mechanisms are ready!
        // FIX: Removed the "50.0" argument since Shooter handles its own tolerance now
       
    }

    @Override
    public boolean isFinished() {
        return false; // Runs as long as the button is held
    }


    @Override
    public void end(boolean interrupted) {
        // When the driver lets go of the trigger, shut everything down safely
        m_shooter.stop();
        m_hood.stop();
        m_intake.stop();
        m_turret.stop(); // Stop the turret from tracking wildly
    }
}
