package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    class GyroIOInputs {
        public GyroIOData gyroData = new GyroIOData(
                false, // gyroConnected
                0,     // headingDeg
                0,     // pitchDeg
                0,     // rollDeg
                0,     // gyroRateRadPerSec
                0,     // pitchRateRadPerSec
                0,     // rollRateRadPerSec
                0,     // accelX
                0,     // accelY
                0      // accelZ
        );
    }

    record GyroIOData(
            // --- Conexión ---
            boolean gyroConnected,

            // --- Orientación ---
            double headingDeg,
            double pitchDeg,
            double rollDeg,

            // --- Velocidades angulares ---
            double gyroRateRadPerSec,
            double pitchRateRadPerSec,
            double rollRateRadPerSec,

            // --- Aceleraciones (m/s²) ---
            double accelX,
            double accelY,
            double accelZ
    ) {}

    /** Actualiza el objeto de inputs con los valores actuales del giroscopio. */
    default void updateInputs(GyroIOInputs inputs) {}

    /**
     * Resetea el yaw del giroscopio.
     *
     * @param yawDeg ángulo al que se quiere resetear (0 para azul, 180 para rojo).
     */
    default void setYaw(double yawDeg) {}

    /** Retorna los inputs actuales del giroscopio. */
    default GyroIOInputs getInputs() {
        return new GyroIOInputs();
    }
}