package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

    @AutoLog
    class SwerveModuleIOInputs {
        public SwerveModuleIOData moduleData = new SwerveModuleIOData(
                false,  // driveConnected
                false,  // turningConnected
                0, 0,   // drive position, velocity
                0, 0,   // turning position, velocity
                0,      // absoluteEncoderRadians
                0, 0,   // appliedVolts drive, turning
                0, 0,   // torqueCurrent drive, turning
                0, 0,   // supplyCurrent drive, turning
                0, 0    // temperature drive, turning
        );
    }

    record SwerveModuleIOData(
            // --- Conexión ---
            boolean driveConnected,
            boolean turningConnected,

            // --- Drive (tracción) ---
            double drivePositionMeters,
            double driveVelocityMetersPerSecond,

            // --- Turning (giro) ---
            double turningPositionRadians,
            double turningVelocityRadPerSecond,

            // --- Encoder absoluto ---
            double absoluteEncoderRadians,

            // --- Voltaje aplicado ---
            double appliedVoltsDrive,
            double appliedVoltsTurning,

            // --- Corriente torque ---
            double torqueCurrentDrive,
            double torqueCurrentTurning,

            // --- Corriente supply ---
            double supplyCurrentDrive,
            double supplyCurrentTurning,

            // --- Temperatura ---
            double temperatureDrive,
            double temperatureTurning
    ) {}

    /** Actualiza el objeto de inputs con los valores actuales del hardware. */
    default void updateInputs(SwerveModuleIOInputs inputs) {}

    /** Setea la velocidad lineal deseada del módulo en metros por segundo. */
    default void setVelocity(double velocityMps) {}

    /** Setea el ángulo deseado del módulo en radianes. */
    default void setAngle(double angleRad) {}

    /** Sincroniza el encoder relativo del giro con el encoder absoluto y resetea el drive. */
    default void resetEncoders(double absoluteEncoderPosition) {}

    /** Detiene ambos motores del módulo inmediatamente. */
    default void stop() {}
}