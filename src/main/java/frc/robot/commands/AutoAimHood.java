package frc.robot.commands;


import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;


public class AutoAimHood extends Command {
    private final Hood m_hood;
    private final NetworkTable m_limelightTable;
   
    // Smooths out the 'ty' vertical angle over 5 frames (100ms)
    // This stops the hood from "vibrating" due to camera noise.
    private final MedianFilter m_tyFilter = new MedianFilter(5);


    public AutoAimHood(Hood hood) {
        m_hood = hood;
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight_turret");
        addRequirements(m_hood);
    }


   @Override
    public void execute() {
        // 1. Check if the Limelight actually sees a target ('tv')
        boolean hasTarget = m_limelightTable.getEntry("tv").getDouble(0) == 1.0;


        // --- NEW: Grab the current Target ID ('tid') from the Limelight ---
        // Defaults to -1.0 if it can't find a tag ID
        double currentTagID = m_limelightTable.getEntry("tid").getDouble(-1.0);
        boolean isCorrectTag = (currentTagID == 9.0);
        // ONLY aim at Tag 9!


        // 2. Only move the hood if we have a target AND it's the correct tag
        if (hasTarget && isCorrectTag) {
            // Get the vertical angle and run it through the filter
            double rawTy = m_limelightTable.getEntry("ty").getDouble(0);
            double filteredTy = m_tyFilter.calculate(rawTy);


            // Distance math: (GoalHeight - CamHeight) / tan(CamAngle + ty)
            double distance = (2.6 - 0.5) / Math.tan(Math.toRadians(30.0 + filteredTy));


            // Send to your existing subsystem method
            m_hood.setAngleFromDistance(distance);
        } else {
            // If target is lost OR it's the wrong tag, stop moving to prevent damage
            m_hood.stop();
        }
    }


    @Override
    public void end(boolean interrupted) {
        m_hood.stop();
        m_tyFilter.reset(); // Clear the filter for next time
    }
}
