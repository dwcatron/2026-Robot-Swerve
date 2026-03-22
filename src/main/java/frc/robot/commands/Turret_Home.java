package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class Turret_Home extends Command {
    private final Turret m_turret;
    private final PIDController m_pid = new PIDController(4, 0, 0);
    private final double TARGET_POS = 0.54355;

    public Turret_Home(Turret turret) {
        m_turret = turret;
        addRequirements(m_turret);
       
        // 1. Tell PID that 0.0 and 1.0 are the same point (wrapping)
        // This forces it to calculate the shortest path to 0.544
        m_pid.enableContinuousInput(0.0, 1.0);
       
        m_pid.setTolerance(0.005);
    }

    @Override
    public void execute() {
        // 2. PID now automatically picks the fastest direction
        double speed = m_pid.calculate(m_turret.getPosition(), TARGET_POS);
       
        m_turret.setSpeed(-speed);
    }

    @Override
    public boolean isFinished() {
        return m_pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }
}
