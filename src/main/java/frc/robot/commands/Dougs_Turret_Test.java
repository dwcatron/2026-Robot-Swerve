    package frc.robot.commands;

    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.subsystems.Turret;
    import frc.robot.LimelightHelpers;

    public class Dougs_Turret_Test extends Command {
    private final Turret m_turret;
    
    // Tuning constant: Start with 0.02. 
    // If it moves AWAY from the target, change this to -0.02
    private final double kP = 0.05; 
    private final String limelightName = "limelight-turret";

    public Dougs_Turret_Test(Turret turret) {
    this.m_turret = turret;
    // Ensures the turret doesn't try to run two commands at once
    addRequirements(m_turret);
    }

    @Override
    public void execute() {
    // 1. Check if "limelight-turret" sees an AprilTag
    boolean hasTarget = LimelightHelpers.getTV(limelightName);

    if (hasTarget) {
    // 2. Get horizontal offset (tx) from the limelight
    double tx = LimelightHelpers.getTX(limelightName);
    double motorSpeed = tx * kP;

    // If we are within 1 degree of center, just stop.
    if ((Math.abs(tx) < 1.0) && Math.abs(tx) > -1.0) {
    m_turret.stop();
    } else {
    m_turret.setSpeed(motorSpeed);
}

            
    // 3. Calculate speed: speed = error * kP
    // Example: if tx is 5 degrees, speed is 0.1 (10% power)
            
    // 4. Send speed to the motor through the subsystem
    m_turret.setSpeed(motorSpeed);
    } else {
    // No target found? Stop moving.
    m_turret.stop();
    }
    }

    @Override
    public void end(boolean interrupted) {
    // Safety: Always stop the motor when the button is released
    m_turret.stop();
    }
    }
