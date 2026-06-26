package frc.robot.subsystems.swerve;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 * Implementación de simulación de {@link SwerveIO} usando MapleSim.
 *
 * <p>Lee el heading, velocidades angulares y aceleraciones directamente
 * desde el {@link SwerveDriveSimulation}, que mantiene el estado físico
 * completo del chasis simulado.
 *
 * <p>Uso típico:
 * <pre>
 *   new Swerve(new GyroIOSim(swerveDriveSimulation), alliance)
 * </pre>
 */
public class GyroIOSim implements GyroIO {

    /** Simulación del chasis completo — fuente de verdad del heading en sim. */
    private final SwerveDriveSimulation driveSimulation;

    /**
     * Crea un giroscopio simulado a partir de la simulación del chasis.
     *
     * @param driveSimulation instancia del {@link SwerveDriveSimulation} compartida
     *                        con los {@code SwerveModuleIOSim} del mismo robot.
     */
    public GyroIOSim(SwerveDriveSimulation driveSimulation) {
        this.driveSimulation = driveSimulation;
    }

    /**
     * Rellena los inputs con los valores actuales de la simulación.
     *
     * <p>En simulación el giroscopio siempre está "conectado" y no hay
     * temperatura ni corriente que reportar.
     */
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.gyroData = new GyroIOData(
                // gyroConnected — siempre true en sim
                true,

                // Heading en grados — obtenido desde la pose simulada del chasis
                driveSimulation.getSimulatedDriveTrainPose()
                        .getRotation()
                        .getDegrees(),

                // Pitch y roll — MapleSim simula en plano 2D, siempre 0
                0.0,
                0.0,

                // Velocidad angular de yaw (rad/s) — del chasis simulado
                driveSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative()
                        .omegaRadiansPerSecond,

                // Pitch rate y roll rate — no simulados en 2D
                0.0,
                0.0,

                // Aceleraciones — no expuestas directamente por MapleSim, reportamos 0
                0.0,
                0.0,
                0.0
        );
    }

    /**
     * Resetea el yaw de la simulación.
     *
     * <p>MapleSim no expone un reset de yaw directo, así que este método
     * es un no-op. El heading se resetea instanciando la simulación con
     * la pose inicial correcta en {@code SimulationContainer}.
     */
    @Override
    public void setYaw(double yawDeg) {
        // No-op: en MapleSim el heading lo controla la pose inicial del SwerveDriveSimulation.
    }
}