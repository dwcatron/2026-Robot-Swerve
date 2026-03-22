 package frc.robot.commands;


    import edu.wpi.first.math.controller.PIDController;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.subsystems.Turret;
    import frc.robot.LimelightHelpers;


    public class Dougs_Turret_Test extends Command {
    private final Turret m_turret;
   
    // Tuning constant: Start with 0.02.
    // If it moves AWAY from the target, change this to -0.02
    private final PIDController m_pid = new PIDController(0.05, 0, 0);
    private final String limelightName = "limelight-turret";


    public Dougs_Turret_Test(Turret turret) {
    this.m_turret = turret;
    // Ensures the turret doesn't try to run two commands at once
    addRequirements(m_turret);
    }


   @Override
public void execute() {
    // 1. Check if "limelight-turret" sees a target
    boolean hasTarget = LimelightHelpers.getTV(limelightName);


    if (hasTarget) {
        // 2. Get horizontal offset (tx). This is our "Measurement"
        double tx = LimelightHelpers.getTX(limelightName);
       
        // 3. Use PID to calculate speed.
        // Goal (Setpoint) is 0 degrees. Current position (Measurement) is tx.
    double motorSpeed = m_pid.calculate(LimelightHelpers.getTX(limelightName), 5);


        // 4. Deadband: If we are within 2 degree, stop to prevent jitter
         if ((Math.abs(tx) < 7.0) && (Math.abs(tx) > 3.0)) {
        m_turret.stop();
        } else {
            m_turret.setSpeed(-motorSpeed);
        }
    }
    if (hasTarget) {


    }
   
    else {
        m_turret.stop();
    }
}




    @Override
    public void end(boolean interrupted) {
    // Safety: Always stop the motor when the button is released
    m_turret.stop();
    }
    }
