package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class FireFuel extends Command {

    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Intake m_intake;
    
    private final DoubleSupplier m_distanceSupplier;
    private final DoubleSupplier m_txSupplier;

    public FireFuel(
            Shooter shooter, 
            Turret turret, 
            Intake intake, 
            DoubleSupplier distanceSupplier, 
            DoubleSupplier txSupplier) {
                
        this.m_shooter = shooter;
        this.m_turret = turret;
        this.m_intake = intake;
        this.m_distanceSupplier = distanceSupplier;
        this.m_txSupplier = txSupplier;

        addRequirements(m_shooter, m_turret, m_intake);
    }

    @Override
    public void initialize() {
        m_turret.startTracking();
    }

    @Override
    public void execute() {
        // 1. Spool the shooter continuously based on live distance
        m_shooter.setRPMFromPhysics(m_distanceSupplier.getAsDouble());

        // 2. Grab the current Target ID from the Limelight
        double currentTagID = NetworkTableInstance.getDefault()
                                .getTable("limelight_turret")
                                .getEntry("tid").getDouble(-1.0);

        // --- NEW: Dynamic REBUILT Alliance Tag Checking ---
        var alliance = DriverStation.getAlliance();
        // Default to Red if the FMS hasn't connected yet, just to be safe
        boolean isRedAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        
        boolean isCorrectTag = false;
        if (isRedAlliance) {
            // RED ALLIANCE: Allow Hub tags 8 (Center), 9 (Edge), or 11 (Edge)
            isCorrectTag = (currentTagID == 8.0 || currentTagID == 9.0 || currentTagID == 11.0);
        } else {
            // BLUE ALLIANCE: Allow Hub tags 24 (Center), 25 (Edge), or 27 (Edge)
            isCorrectTag = (currentTagID == 24.0 || currentTagID == 25.0 || currentTagID == 27.0);
        }

        // 3. Check if the robot is ready to fire
        boolean turretLocked = Math.abs(m_txSupplier.getAsDouble()) < constants.kTurretLockToleranceDeg;
        boolean shooterReady = m_shooter.isAtSpeed();

        // 4. Feed the fuel ONLY if locked, up to speed, and looking at valid REBUILT tags
        if (turretLocked && shooterReady && isCorrectTag) {
            // Intake feeds the shooter directly now
            m_intake.setSpeed(-0.9, 0.9); 
        } else {
            // Pause feeding if not ready, wrong tag, or wrong alliance
            m_intake.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_intake.stop();
        
        m_turret.stop();
        m_turret.centerTurret(); 
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously while the trigger is held
    }
}