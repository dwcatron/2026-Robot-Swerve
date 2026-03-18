package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    private final SparkMax turretMotor = new SparkMax(10, MotorType.kBrushless);
    private final CANcoder turretEncoder = new CANcoder(20);

    private final double MIN_POSITION = 0.0;
    private final double MAX_POSITION = 1;

    public Turret() {

        SparkMaxConfig config = new SparkMaxConfig();

        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);

        turretMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public double getPosition() {
        return turretEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setSpeed(double speed) {
        double position = getPosition();

        if ((position <= MIN_POSITION && speed < 0) ||
            (position >= MAX_POSITION && speed > 0)) {
            turretMotor.set(0);
        } else {
            turretMotor.set(speed);
        }
    }

    public void stop() {
        turretMotor.set(0);
    }
}