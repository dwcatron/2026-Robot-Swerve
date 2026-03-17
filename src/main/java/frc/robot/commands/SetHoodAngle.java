package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;


public class SetHoodAngle extends Command {
    private final Hood m_hood;
    private final double m_angle;


    /**
     * Creates a new command to set the hood angle.
     * * @param hood  The hood subsystem
     * @param angle The target angle in degrees
     */
    public SetHoodAngle(Hood hood, double angle) {
        this.m_hood = hood;
        this.m_angle = angle;
       
        // Require the hood so no two commands try to move it at once
        addRequirements(m_hood);
    }


    @Override
    public void initialize() {
        // Tell the hood to start moving to the target
        m_hood.setAngle(m_angle);
    }


    @Override
    public boolean isFinished() {
        // This command finishes once we are within 0.5 degrees of the target
        return m_hood.isAtPosition(m_angle, 0.5);
    }
}
