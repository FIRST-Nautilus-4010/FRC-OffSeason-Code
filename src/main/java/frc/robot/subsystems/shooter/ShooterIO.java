package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {
        public ShooterIOData shooterData = new ShooterIOData(false, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    record ShooterIOData(
        boolean motorConnected,
        double positionLeft,
        double positionRight,
        double velocityLeft,
        double velocityRight,
        double appliedVoltsLeft,
        double appliedVoltsRight,
        double supplyCurrentLeft,
        double supplyCurrentRight
    ) {}

    default void updateInputs(ShooterIOInputs inputs) {}

    default void setVelocity(double velocityRps) {}

    default void setVoltage(double volts) {}

    default void runOpenLoop(double output) {}

    default void stop() {}
}