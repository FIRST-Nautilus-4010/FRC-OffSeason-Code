package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

/**
 * Representa un módulo swerve completo:
 * <ul>
 *   <li>Hardware (motores + encoder) a través de {@link SwerveModuIeIOHardware}</li>
 *   <li>Control de velocidad y ángulo vía {@link SwerveController}</li>
 *   <li>Lógica de limitación de aceleraciones para mejorar estabilidad</li>
 * </ul>
 */
public class SwerveModule {

    /** Acceso al hardware del módulo (motores, encoder, etc.). */
    private final SwerveModuleIO io;

    /** Última vez que se enviaron datos de debug a SmartDashboard. */

    /** Proveedor de la altura del centro de masa del robot (m). */
    private static Supplier<Double> massCenterHeight = () -> 0.0;

    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();


    /**
     * Crea un módulo swerve.
     *
     * @param driveTalonFxId    ID CAN del TalonFX de tracción
     * @param turningTalonFxId  ID CAN del TalonFX de giro
     * @param absoluteEncoderId ID CAN del encoder absoluto del módulo
     */
    public SwerveModule(SwerveModuleIO io) {
        this.io = io;
    }

    /**
     * Define la función que devuelve la altura del centro de masa del robot.
     * <p>
     * No se usa aún en la lógica actual, pero es útil para futuros cálculos
     * de estabilidad (vuelco, transferencia de carga, etc.).
     *
     * @param supplier proveedor de la altura del centro de masa (m)
     */
    public static void setMassCenterHeightSupplier(Supplier<Double> supplier) {
        massCenterHeight = supplier;
    }

    /**
     * Estado actual del módulo (velocidad y ángulo).
     *
     * @return {@link SwerveModuleState} con:
     *         <ul>
     *           <li>Velocidad lineal en m/s</li>
     *           <li>Ángulo del módulo como {@link Rotation2d}</li>
     *         </ul>
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.moduleData.driveVelocityMetersPerSecond(),
            new Rotation2d(inputs.moduleData.turningPositionRadians())
        );
    }

    /**
     * Posición actual del módulo (distancia recorrida + ángulo).
     *
     * @return {@link SwerveModulePosition} con:
     *         <ul>
     *           <li>Distancia en metros desde el último reset de encoders</li>
     *           <li>Ángulo actual del módulo</li>
     *         </ul>
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.moduleData.drivePositionMeters(),
            new Rotation2d(inputs.moduleData.turningPositionRadians())
        );
    }

    /** Detiene completamente el módulo (drive y steer). */
    public void stop() {
        io.stop();
    }

    public void periodic(String moduleName) {
        io.updateInputs(inputs);

        Logger.runEveryN(10, () -> Logger.processInputs("Swerve/Module/" + moduleName, inputs));
    }

    /**
     * Aplica un estado deseado al módulo, respetando limitaciones de aceleración
     * para mejorar tracción y estabilidad.
     *
     * @param desiredState estado objetivo (velocidad y ángulo)
     * @param chassisRoll  roll actual del chasis (grados)
     * @param chassisPitch pitch actual del chasis (grados)
     */
    public void setDesiredState(SwerveModuleState desiredState, double chassisRoll, double chassisPitch, boolean accelerationLimitsDisabled) {
        if (desiredState.speedMetersPerSecond < 0.001) {
            io.stop();
            return;
        }

        // Ángulo actual del módulo medido por el encoder absoluto.
        Rotation2d encoderRotation = Rotation2d.fromRadians(inputs.moduleData.absoluteEncoderRadians());

        if (!accelerationLimitsDisabled) {
            double desiredFinalVel = desiredState.speedMetersPerSecond;

            // Velocidad objetivo y actual (en m/s).
            double currentVel = Math.abs(inputs.moduleData.driveVelocityMetersPerSecond());

            // Dirección (ángulo) objetivo en radianes.
            double wantedDirection = desiredState.angle.getRadians();

            // Aceleración teórica necesaria para pasar de velocidad actual a deseada en un ciclo.
            double wantedAcc = (desiredFinalVel - currentVel) / Timer.getFPGATimestamp();

            // Limita la aceleración en función de capacidades del robot y estabilidad.
            double[] accLimits = accLimits(wantedAcc, wantedDirection, desiredFinalVel);

            // Si quieres activar estabilidad extra, descomenta: (aún en fase de pruebas)
            // accLimits = applyStabilityAssist(accLimits[0], accLimits[1], chassisRoll, chassisPitch);

            double limitedAcc = accLimits[0];
            double limitedDirection = accLimits[1];

            // Calcula la velocidad en el siguiente ciclo usando la aceleración limitada.
            double nextWantedVel = currentVel + (limitedAcc * Timer.getFPGATimestamp());

            // Crea un nuevo estado con velocidad y dirección ya limitadas.
            SwerveModuleState optimizedState =
                    new SwerveModuleState(nextWantedVel, Rotation2d.fromRadians(limitedDirection));

            // Optimiza para minimizar el giro del módulo (puede invertir la rueda).
            optimizedState.optimize(encoderRotation);

            io.setVelocity(optimizedState.speedMetersPerSecond);
            io.setAngle(optimizedState.angle.getRadians());
        } else {
            desiredState.optimize(encoderRotation);
            io.setVelocity(desiredState.speedMetersPerSecond);
            io.setAngle(desiredState.angle.getRadians());
        }
    }

    /**
     * Limita la aceleración deseada según:
     * <ul>
     *   <li>Aceleración máxima hacia adelante (por tracción)</li>
     *   <li>Aceleración máxima lateral (por skid)</li>
     *   <li>Dirección resultante después de aplicar los límites</li>
     * </ul>
     *
     * @param wantedAcc       aceleración deseada (m/s²) con signo
     * @param wantedDirection dirección deseada (rad)
     * @return array [aceleraciónLimitada (m/s²), direcciónLimitada (rad)]
     */
    private double[] accLimits(double wantedAcc, double wantedDirection, double desiredFinalVel) {
        double wantedAccMagnitude = Math.abs(wantedAcc);

        // Aceleración máxima hacia adelante: se reduce cuando te acercas a la velocidad máxima.
        double maxForwardAccel =
                SwerveConfig.MAX_FORDWARD_ACCEL *
                (1.0 - (inputs.moduleData.driveVelocityMetersPerSecond() / SwerveConfig.MAX_VELOCITY_METERS_PER_SECOND));

        double forwardAccel = Math.min(wantedAccMagnitude, maxForwardAccel);
        double skidAccel = Math.min(wantedAccMagnitude, SwerveConfig.MAX_SKID_ACCEL);

        // Usa el límite más restrictivo.
        double minAccel = Math.min(skidAccel, forwardAccel);

        // Descompone la aceleración en componentes frontal/lateral según el ángulo deseado.
        double wantedSideAcc = minAccel * -Math.sin(wantedDirection);
        double wantedFrontAcc = minAccel * Math.cos(wantedDirection);

        // Aplica límites independientes a cada eje.
        double limitedFrontAcc = clamp(
                wantedFrontAcc,
                -SwerveConfig.MAX_FRONT_ACCEL,
                SwerveConfig.MAX_FRONT_ACCEL
        );
        double limitedSideAcc = clamp(
                wantedSideAcc,
                -SwerveConfig.MAX_SIDE_ACCEL,
                SwerveConfig.MAX_SIDE_ACCEL
        );

        // Magnitud final de aceleración limitada.
        double limitedAcc = Math.hypot(limitedFrontAcc, limitedSideAcc);

        // Calcula dirección resultante.
        final double limitedDirection;
        if (Math.abs(limitedSideAcc) < SwerveConfig.MAX_SIDE_ACCEL
                && Math.abs(limitedFrontAcc) < SwerveConfig.MAX_FRONT_ACCEL) {
            // Dentro de límites en ambos ejes: conserva la dirección original.
            limitedDirection = wantedDirection;
        } else {
            // Dirección basada en las componentes saturadas.
            limitedDirection = Math.atan2(-limitedSideAcc, limitedFrontAcc);
        }

        // Mantén el signo de la aceleración original.
        return new double[] { Math.copySign(limitedAcc, wantedAcc), limitedDirection };
    }

    /**
     * Aplica una asistencia extra para mejorar estabilidad ante grandes ángulos
     * de roll/pitch (para evitar vuelcos).
     *
     * @param accHypot      magnitud de la aceleración (m/s²)
     * @param accAngle      dirección de la aceleración (rad)
     * @param chassisRoll   roll del chasis (grados)
     * @param chassisPitch  pitch del chasis (grados)
     * @return array [aceleraciónNueva (m/s²), direcciónNueva (rad)]
     */
    private double[] applyStabilityAssist(double accHypot, double accAngle,
                                          double chassisRoll, double chassisPitch) {

        // Ángulo de chasis a partir del cual empezamos a intervenir fuertemente.
        final double maxSafeAngleDeg = 5.0;

        // 0 = el asistente domina por completo, 1 = casi apagado.
        final double kAssist = 0.3;

        double accX = accHypot * Math.cos(accAngle);
        double accY = accHypot * Math.sin(accAngle);

        double assistX = 0.0;
        double assistY = 0.0;

        // Corrige pitch (adelante/atrás).
        if (Math.abs(chassisPitch) > maxSafeAngleDeg) {
            accX *= kAssist;
            assistX =
                    -Math.signum(chassisPitch)
                    * ((Math.abs(chassisPitch) - maxSafeAngleDeg) / 80.0)
                    * (1.0 - kAssist)
                    * (SwerveConfig.MAX_SKID_ACCEL - accHypot)
                    * Math.cos(accAngle);
        }

        // Corrige roll (izquierda/derecha).
        if (Math.abs(chassisRoll) > maxSafeAngleDeg) {
            accY *= kAssist;
            assistY =
                    -Math.signum(chassisRoll)
                    * ((Math.abs(chassisRoll) - maxSafeAngleDeg) / 80.0)
                    * (1.0 - kAssist)
                    * (SwerveConfig.MAX_SKID_ACCEL - accHypot)
                    * Math.sin(accAngle);
        }

        accX += assistX;
        accY += assistY;

        double newAccHypot = Math.hypot(accX, accY);
        double newAccAngle = Math.atan2(accY, accX);

        return new double[] { newAccHypot, newAccAngle };
    }

    /** Pequeño helper para limitar un valor a un rango. */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
}
