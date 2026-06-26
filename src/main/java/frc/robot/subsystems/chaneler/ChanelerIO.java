package frc.robot.subsystems.chaneler;

import org.littletonrobotics.junction.AutoLog;

public interface ChanelerIO {
    @AutoLog
    class ChanelerIOInputs {
        public ChanelerIOData ChanelerData = new ChanelerIOData(false, 0, 0, 0, 0, 0);
    }

    record ChanelerIOData(
        boolean motorConnected,
        double velocity,
        double appliedVolts,
        double torqueCurrent,
        double supplyCurrent,
        double temperature
    ) {}

    default void updateInputs(ChanelerIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void runOpenLoop(double output) {}

    default void setVelocity(double velocityRps) {}

    default void stop() {}
}
