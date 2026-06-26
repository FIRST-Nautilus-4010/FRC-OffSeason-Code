package frc.robot.utils;

import frc.robot.subsystems.swerve.Swerve;

/**
 * Rastrea la "confianza" en la odometría del robot.
 *
 * <p>Se basa en:</p>
 * <ul>
 *   <li>Diferencia entre velocidad media de ruedas y velocidad del chasis
 *       para detectar skid (patinaje).</li>
 *   <li>Diferencia entre pose de visión y odometría para decidir si
 *       confiar en la medición de cámara.</li>
 * </ul>
 */
public class PoseConfidenceTracker {

    /** Indica si el robot está probablemente patinando (skid). */
    private boolean skidding = false;

    /**
     * Umbral de diferencia entre velocidad media de rueda y velocidad
     * del chasis a partir del cual se considera que hay skid (m/s).
     */
    private static final double SKID_SPEED_DIFF_THRESHOLD = 0.5;


    /**
     * Actualiza el estado interno de skid en base al subsistema swerve.
     *
     * @param swerve subsistema swerve del que se leen velocidades
     */
    public void update(Swerve swerve) {
        double avgWheelSpeed = swerve.getAverageWheelSpeed();
        double chassisSpeed = swerve.getChassisSpeedMagnitude();

        // Si las ruedas "dicen" ir mucho más rápido que el chasis, probablemente hay skid.
        skidding = Math.abs(avgWheelSpeed - chassisSpeed) > SKID_SPEED_DIFF_THRESHOLD;
    }

    /** @return {@code true} si se detecta posible skid (poca confianza en odometría). */
    public boolean isSkidding() {
        return skidding;
    }
}
