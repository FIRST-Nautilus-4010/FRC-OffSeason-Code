package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    class IndexerIOInputs {
        public IndexerIOData indexerData = new IndexerIOData(false, false, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    record IndexerIOData(
        boolean motorLeftConnected,
        boolean motorRightConnected,
        double velocityLeft,
        double velocityRight,
        double appliedVoltsLeft,
        double appliedVoltsRight,
        double torqueCurrentLeft,
        double torqueCurrentRight,
        double supplyCurrentLeft,
        double supplyCurrentRight
    ) {}

    default void updateInputs(IndexerIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void runOpenLoop(double output) {}

    default void setVelocity(double velocityRps) {}

    default void stop() {}
}