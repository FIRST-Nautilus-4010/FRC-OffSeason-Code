package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

/**
 * Implementación de simulación de {@link SwerveModuleIO} usando MapleSim.
 *
 * <p>Usa {@link SwerveModuleSimulation} para simular la física realista del módulo:
 * dinámica de motores, encoders y fricción de rueda.
 *
 * <p>El control se implementa con PIDControllers de WPILib + SimpleMotorFeedforward.
 * Las ganancias de {@link SwerveConfig} están tuneadas para el TalonFX en unidades
 * de rotaciones/rps, por lo que se escalan por (1 / 2π) para convertirlas a
 * radianes/rad/s, que es lo que usa MapleSim.
 *
 * <p>Uso típico: instanciar pasando uno de los módulos del {@code SwerveDriveSimulation}:
 * <pre>
 *   new SwerveModuleIOSim(swerveDriveSimulation.getModules()[0])
 * </pre>
 */
public class SwerveModuleIOSim implements SwerveModuleIO {

    // -----------------------------------------------------------------------
    // SIMULACIÓN MAPLE-SIM
    // -----------------------------------------------------------------------

    /** Módulo swerve simulado por MapleSim. Contiene la física completa del módulo. */
    private final SwerveModuleSimulation moduleSimulation;

    /**
     * Controlador de motor simulado para tracción (drive).
     * Recibe demandas de voltaje y simula la respuesta del motor.
     */
    private final SimulatedMotorController.GenericMotorController driveMotor;

    /**
     * Controlador de motor simulado para giro (steer).
     * Recibe demandas de voltaje y simula la respuesta del motor.
     */
    private final SimulatedMotorController.GenericMotorController steerMotor;

    // -----------------------------------------------------------------------
    // CONTROL SOFTWARE
    // -----------------------------------------------------------------------

    /**
     * PID de velocidad para el motor de tracción.
     * Ganancias escaladas de rps (TalonFX) a rad/s (MapleSim): kX_sim = kX / (2π).
     */
    private final PIDController drivePID;

    /**
     * Feedforward para el motor de tracción.
     * kV y kA escalados de V/rps y V/(rps/s) a V/(rad/s) y V/(rad/s²).
     */
    private final SimpleMotorFeedforward driveFF;

    /**
     * PID de posición para el motor de giro.
     * Ganancias escaladas de rotaciones (TalonFX) a radianes (MapleSim): kX_sim = kX / (2π).
     */
    private final PIDController steerPID;

    // -----------------------------------------------------------------------
    // ESTADO INTERNO
    // -----------------------------------------------------------------------

    /** Voltaje actualmente aplicado al motor de tracción (para telemetría). */
    private double driveAppliedVolts = 0.0;

    /** Voltaje actualmente aplicado al motor de giro (para telemetría). */
    private double steerAppliedVolts = 0.0;

    /** Indica si los motores están activos (false = detenidos con stop()). */
    private boolean enabled = true;

    private double steerTargetAngle = 0.0;

    // -----------------------------------------------------------------------
    // CONSTRUCTOR
    // -----------------------------------------------------------------------

    /**
     * Crea una implementación de simulación para un módulo swerve.
     *
     * @param moduleSimulation referencia al módulo simulado por MapleSim,
     *                         obtenido con {@code swerveDriveSimulation.getModules()[i]}.
     */
    public SwerveModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;

        this.driveMotor = moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(120));

        this.steerMotor = moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(40));

        this.drivePID = new PIDController(
                SwerveConfig.VEL_KP / (2.0 * Math.PI),
                SwerveConfig.VEL_KI / (2.0 * Math.PI),
                SwerveConfig.VEL_KD / (2.0 * Math.PI)
        );

        this.driveFF = new SimpleMotorFeedforward(
                SwerveConfig.VEL_KS,
                SwerveConfig.VEL_KV / (2.0 * Math.PI),
                SwerveConfig.VEL_KA / (2.0 * Math.PI)
        );

        this.steerPID = new PIDController(
                SwerveConfig.POS_KP / (2.0 * Math.PI),
                SwerveConfig.POS_KI / (2.0 * Math.PI),
                SwerveConfig.POS_KD / (2.0 * Math.PI)
        );
        this.steerPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // -----------------------------------------------------------------------
    // UPDATE INPUTS (AdvantageKit)
    // -----------------------------------------------------------------------

    /**
     * Rellena el record de inputs con los valores del módulo simulado.
     *
     * <p>En simulación ambos motores siempre están "conectados" (no hay CAN bus),
     * por lo que {@code driveConnected} y {@code turningConnected} son siempre {@code true}.
     */
    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        double currentSteerAngle = moduleSimulation.getSteerAbsoluteFacing().getRadians();
        steerAppliedVolts = steerPID.calculate(currentSteerAngle, steerTargetAngle);
        steerMotor.requestVoltage(Volts.of(steerAppliedVolts));

        inputs.moduleData = new SwerveModuleIOData(
                true,
                true,

                moduleSimulation.getDriveWheelFinalPosition().in(Radians)
                        * (SwerveConfig.WHEEL_DIAMETER / 2.0),

                moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond)
                        * (SwerveConfig.WHEEL_DIAMETER / 2.0),

                moduleSimulation.getSteerAbsoluteFacing().getRadians(),
                moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond),

                moduleSimulation.getSteerAbsoluteFacing().getRadians(),

                driveAppliedVolts,
                steerAppliedVolts,

                moduleSimulation.getDriveMotorStatorCurrent().in(Amps),
                moduleSimulation.getSteerMotorStatorCurrent().in(Amps),

                moduleSimulation.getDriveMotorStatorCurrent().in(Amps),
                moduleSimulation.getSteerMotorStatorCurrent().in(Amps),

                0.0,
                0.0
        );
    }

    // -----------------------------------------------------------------------
    // COMANDOS DE CONTROL
    // -----------------------------------------------------------------------

    /**
     * Establece la velocidad lineal deseada del módulo en m/s.
     *
     * <p>Convierte m/s a rad/s de rueda, calcula el voltaje con PID + FF
     * (ambos en unidades de rad/s) y lo envía al motor simulado.
     */
    @Override
    public void setVelocity(double velocityMps) {
        if (!enabled) return;

        double wheelRadPerSec = velocityMps / (SwerveConfig.WHEEL_DIAMETER / 2.0);

        double currentWheelRadPerSec = moduleSimulation
                .getDriveWheelFinalSpeed()
                .in(RadiansPerSecond);

        double pidOutput = drivePID.calculate(currentWheelRadPerSec, wheelRadPerSec);
        double ffOutput  = driveFF.calculate(wheelRadPerSec);
        driveAppliedVolts = pidOutput + ffOutput;

        driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    }

    /**
     * Establece el ángulo deseado del módulo en radianes.
     *
     * <p>Usa el PID de posición con continuous input para girar
     * por el camino más corto.
     */
    @Override
    public void setAngle(double angleRad) {
        steerTargetAngle = angleRad;
    }

    /**
     * Resetea los encoders — no-op en simulación.
     * MapleSim gestiona el estado del encoder internamente.
     */
    @Override
    public void resetEncoders(double absoluteEncoderPosition) {
        // No-op en simulación.
    }

    /**
     * Detiene ambos motores inmediatamente.
     */
    @Override
    public void stop() {
        enabled = false;
        driveAppliedVolts = 0.0;
        steerAppliedVolts = 0.0;
        driveMotor.requestVoltage(Volts.of(0));
        steerMotor.requestVoltage(Volts.of(0));
        enabled = true;
    }
}