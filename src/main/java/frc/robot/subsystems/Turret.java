package frc.robot.subsystems;
import frc.robot.generated.constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends SubsystemBase {
    private final SparkMax turretMotor = new SparkMax(constants.kTurretCanId, MotorType.kBrushless);
    private final CANcoder turretEncoder = new CANcoder(constants.kTurretEncoderCanId);

    // 2026 Tip: Use Shuffleboard to find these exact numbers for your 'REBUILT' robot
    private final double MIN_POSITION = 0.15; 
    private final double MAX_POSITION = 0.85;

    public Turret() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getPosition() {
        // Returns rotations (0.0 to 1.0)
        return turretEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setSpeed(double speed) {
        double position = getPosition();
        if ((position <= MIN_POSITION && speed < 0) || (position >= MAX_POSITION && speed > 0)) {
            turretMotor.set(0);
        } else {
            turretMotor.set(speed);
        }
    }

    @Override
    public void periodic() {
        // This is your best friend for debugging during the season
        SmartDashboard.putNumber("Turret Position", getPosition());
    }

    public void stop() { turretMotor.set(0); }
}
