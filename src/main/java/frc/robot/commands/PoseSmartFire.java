package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.generated.constants;
import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

public class PoseSmartFire extends Command {

    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Intake m_intake;
    private final Indexer m_indexer;
    private final CommandSwerveDrivetrain m_drivetrain;

    private final DoubleSupplier m_distanceSupplier;
    private final BooleanSupplier m_smartEnabled;

    private boolean m_preAimed = false;

    public PoseSmartFire(
            Shooter shooter,
            Turret turret,
            Intake intake,
            Indexer indexer,
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier distanceSupplier,
            BooleanSupplier smartEnabled) {

        m_shooter = shooter;
        m_turret = turret;
        m_intake = intake;
        m_indexer = indexer;
        m_drivetrain = drivetrain;
        m_distanceSupplier = distanceSupplier;
        m_smartEnabled = smartEnabled;
            
        addRequirements(m_indexer, m_intake, m_shooter, m_turret);
        
    }

    @Override
    public void initialize() {
        m_preAimed = false;
    }

    @Override
    public void execute() {

        // =============================
        // 1. Pre-aim once using robot pose
        // =============================
        if (!m_preAimed) {

            Pose2d robotPose = m_drivetrain.getState().Pose;

            Translation2d target;

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() &&
                alliance.get() == DriverStation.Alliance.Red) {

                target = constants.kRedHub;
            } else {
                target = constants.kBlueHub;
            }

            Translation2d delta =
                    target.minus(robotPose.getTranslation());

            double fieldAngle =
                    Math.atan2(delta.getY(), delta.getX());

            double turretAngle =
                    Math.toDegrees(fieldAngle)
                    - robotPose.getRotation().getDegrees();

            m_turret.setAngle(turretAngle);
            System.out.println("Turret PreAim: " + turretAngle);
            System.out.println("Robot Heading: " +
            robotPose.getRotation().getDegrees());
            m_preAimed = true;
        }

        // =============================
        // 2. Vision refinement
        // =============================
        double tv = NetworkTableInstance.getDefault()
                .getTable("limelight_turret")
                .getEntry("tv")
                .getDouble(0.0);

        double tx = NetworkTableInstance.getDefault()
                .getTable("limelight_turret")
                .getEntry("tx")
                .getDouble(0.0);

        if (tv == 1.0) {
            double corrected =
                    m_turret.getAngle() + tx;

            m_turret.setAngle(corrected);
        }

        // =============================
        // 3. Shooter RPM from distance
        // =============================
        double distance = m_distanceSupplier.getAsDouble();
        m_shooter.setRPMFromDistance(distance);

        // =============================
        // 4. Feeding logic
        // =============================
        if (!m_smartEnabled.getAsBoolean()) {
            feed();
            return;
        }

        boolean turretLocked =
                Math.abs(tx) < constants.kTurretLockToleranceDeg;

        boolean shooterReady =
                m_shooter.isAtSpeed();

        if (tv == 1.0 &&
            (!constants.kRequireTurretLock || turretLocked) &&
            (!constants.kRequireShooterStable || shooterReady)) {

            feed();
        } else {
            stopFeed();
        }
    }

    private void feed() {
        m_indexer.setPercent(0.7);
        m_intake.setSpeed(-0.9, 0.9);
    }

    private void stopFeed() {
        m_indexer.stop();
        m_intake.stop();
    }

    @Override
    public void end(boolean interrupted) {
        stopFeed();
        m_shooter.stop();
        m_turret.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}

