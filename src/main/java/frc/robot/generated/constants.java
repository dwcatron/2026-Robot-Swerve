package frc.robot.generated;

import edu.wpi.first.units.measure.Distance;

public class constants {
     //Intake
     public static final int kFrontIntakeCanId = 20;
     public static final int kRearIntakeCanId = 25;
     

    //Shooter
     public static final int kShooter_LeftCanId = 21;
     public static final int kShooter_RightCanId = 22;

     //Hood
     public static final int kHoodEncoderCanId = 30;
     public static final int kHoodCanId = 29;
     public static final double kHoodOffset = 17.2265625; // Adjust once the hood is working and we know what zero is
     //Turret
     public static final int kTurretCanId = 28;
     public static final int kTurretEncoderCanId = 31;
     public static final double kTurretMinAngle = -90.0; // Degrees
     public static final double kTurretMaxAngle = 90.0;  // Degrees
     public static final double kTurretOffset = -136.328125; // rawDegrees when turret points perfectly forward

    //Hopper
     public static final int kHopper_LeftCanId = 23;
     public static final int kHopper_RightCanId = 24;
     public static final int Hop_Inches = 12;

    //Climber
     public static final int kClimber_LeftCanId = 26;
     public static final int kClimber_RightCanId = 27;
     // Inside constants.java
     public static final int kIndexerCanId = 34;

     // Shooter PID Constants
    public static final double kShooter_kP = 0.11; // Adjust if it's too slow to reach speed
    public static final double kShooter_kV = 0.12; // This handles the "Feed Forward" (Volts per RPS)

    
    // The Answer to Life, the Universe, and Everything
    public static final int kTheMeaningOfLife = 42; 
    
}

