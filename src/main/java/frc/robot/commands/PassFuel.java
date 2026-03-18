package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;


public class PassFuel extends Command {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Hood m_hood;
    private final Intake m_intake;
   
    // We need the drivetrain to tell us where the robot is on the field!
    private final Supplier<Pose2d> m_poseSupplier;


    // --- PASS SHOT CONSTANTS ---
    private final double kPassRPM = 2500.0;
    private final double kPassHoodAngle = 35.0;


    // The (X, Y) coordinates of where you want to pass the fuel (in meters).
    // You will need to measure this on your practice field!
    private final Translation2d kBlueAllianceDropZone = new Translation2d(2.0, 5.5);
    private final Translation2d kRedAllianceDropZone = new Translation2d(14.5, 5.5);


    public PassFuel(Shooter shooter, Turret turret, Hood hood, Intake intake, Supplier<Pose2d> poseSupplier) {
        m_shooter = shooter;
        m_turret = turret;
        m_hood = hood;
        m_intake = intake;
        m_poseSupplier = poseSupplier;


        addRequirements(shooter, turret, hood, intake);
    }


    @Override
    public void initialize() {
        m_hood.setAngle(kPassHoodAngle);
        m_shooter.setRPM(kPassRPM);
    }


    @Override
    public void execute() {
        // 1. Where are we right now?
        Pose2d currentPose = m_poseSupplier.get();


        // 2. Which side of the field are we throwing to?
        boolean isRedAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        Translation2d targetZone = isRedAlliance ? kRedAllianceDropZone : kBlueAllianceDropZone;


        // 3. Calculate the absolute angle from the robot to the drop zone
        double deltaX = targetZone.getX() - currentPose.getX();
        double deltaY = targetZone.getY() - currentPose.getY();
        Rotation2d fieldAngleToTarget = new Rotation2d(Math.atan2(deltaY, deltaX));


        // 4. Subtract the robot's current heading to get the turret angle relative to the chassis
        Rotation2d turretTargetAngle = fieldAngleToTarget.minus(currentPose.getRotation());
       
        // Command the turret to constantly track this angle
       // m_turret.setAngle(turretTargetAngle.getDegrees());


        // 5. THE PRE-FLIGHT CHECKLIST
       
    }


    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_hood.stop();
        m_intake.stop();
        m_turret.stop();
    }
}
