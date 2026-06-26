package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public IntakeIOData intakeData = new IntakeIOData(false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    record IntakeIOData(
        boolean motorConnected,
        double positionPivot,
        double velocityPivot,
        double positionSpin,
        double velocitySpin,
        double appliedVoltsPivotLeft,
        double appliedVoltsPivotRight,
        double appliedVoltsSpinLeft,
        double appliedVoltsSpinRight,
        double torqueCurrentPivotLeft,
        double torqueCurrentPivotRight,
        double torqueCurrentSpinLeft,
        double torqueCurrentSpinRight,
        double supplyCurrentPivotLeft,
        double supplyCurrentPivotRight,
        double supplyCurrentSpinLeft,
        double supplyCurrentSpinRight,
        double temperaturePivotLeft,
        double temperaturePivotRight,
        double temperatureSpinLeft,
        double temperatureSpinRight
    ) {}

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setPivotVoltage(double volts) {}

    default void setSpinVoltage(double volts) {}

    default void runOpenLoopPivot(double output) {}

    default void runOpenLoopSpin(double output) {}
    
    default void setPosition(double positionRad) {}

    default void setVelocity(double velocityRps) {}

    default void stop() {}
}