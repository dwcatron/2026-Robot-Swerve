package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;
import java.util.function.DoubleSupplier; // 1. IMPORT THIS


public class TurretTrackTarget extends Command {
    private final Turret m_turret;
    private final DoubleSupplier m_angleSupplier; // 2. ADD THIS FIELD
    private final RobotContainer m_container; // 2. ADD THIS FIELD

    // 3. UPDATE CONSTRUCTOR TO ACCEPT THE SUPPLIER
    public TurretTrackTarget(Turret turret, DoubleSupplier angleSupplier, RobotContainer container) {
        m_turret = turret;
        m_angleSupplier = angleSupplier;
        m_container = container;
        addRequirements(m_turret);
    }

    @Override
public void initialize() {
    // Tell Limelight to use the pipeline that tracks the correct target
    // Assume 0 is the tag for your alliance
    m_container.setLimelightPipeline(0);
}
    @Override
    public void execute() {
        // 4. USE THE SUPPLIER TO GET THE ANGLE
        double targetAngle = m_angleSupplier.getAsDouble();
        
        // 5. USE PID TO AIM (Replaces the manual P-loop)
        m_turret.setAngle(targetAngle);
    }

 @Override
    public void end(boolean interrupted) {
        m_turret.setSpeed(0); // Safely stops the motor and disables closed-loop
    }
}
