package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.generated.constants;

public class Shooter extends SubsystemBase {

    private final TalonFX m_leftMotor =
            new TalonFX(constants.kShooter_LeftCanId);

    private final TalonFX m_rightMotor =
            new TalonFX(constants.kShooter_RightCanId);

    private final VelocityVoltage m_velocityRequest =
            new VelocityVoltage(0);

    private double m_targetRPM = 0;

    private final InterpolatingDoubleTreeMap m_rpmMap =
            new InterpolatingDoubleTreeMap();

    public Shooter() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Slot0.kP = constants.kShooter_kP;
        config.Slot0.kV = constants.kShooter_kV;

        m_leftMotor.getConfigurator().apply(config);

        m_rightMotor.setControl(
                new Follower(
                        m_leftMotor.getDeviceID(),
                        MotorAlignmentValue.Opposed));

         /*  Distance → RPM map (tune these)
        m_rpmMap.put(1.5, 2800.0);
        m_rpmMap.put(2.0, 3200.0);
        m_rpmMap.put(2.5, 3600.0);
        m_rpmMap.put(3.0, 4100.0);
        m_rpmMap.put(3.5, 4600.0);*/
    }

    public void setRPM(double targetRPM) {

        m_targetRPM = targetRPM;

        double targetRPS = targetRPM / 60.0;

        m_leftMotor.setControl(
                m_velocityRequest.withVelocity(targetRPS));
    }

    public void setRPMFromDistance(double distanceMeters) {

        distanceMeters = MathUtil.clamp(distanceMeters, 1.2, 4.0);

        double rpm = m_rpmMap.get(distanceMeters);

        setRPM(rpm);
    }

    public double getCurrentRPM() {
        return m_leftMotor.getVelocity()
                .refresh()
                .getValueAsDouble() * 60.0;
    }

    public boolean isAtSpeed() {
        return Math.abs(getCurrentRPM() - m_targetRPM)
                < constants.kShooterToleranceRPM;
    }

    public void stop() {
        m_targetRPM = 0;
        m_leftMotor.stopMotor();
    }
}
