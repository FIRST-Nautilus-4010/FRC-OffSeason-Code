package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.utils.PhoenixUtil;

/**
 * Implementación hardware de {@link GyroIO} usando el Pigeon2.
 *
 * <p>Sigue el mismo patrón de {@code IntakeIOHardware}:
 * StatusSignals con frecuencias configuradas, {@code optimizeBusUtilization}
 * y registro de señales con AdvantageKit.
 */
public class GyroIOHardware implements GyroIO {

    // -----------------------------------------------------------------------
    // HARDWARE
    // -----------------------------------------------------------------------

        /** IMU principal del robot. */
        private final Pigeon2 pigeon;

    // -----------------------------------------------------------------------
    // STATUS SIGNALS
    // -----------------------------------------------------------------------

    // Orientación
    private final StatusSignal<Angle>           yaw;
    private final StatusSignal<Angle>           pitch;
    private final StatusSignal<Angle>           roll;

    // Velocidades angulares
    private final StatusSignal<AngularVelocity> yawRate;
    private final StatusSignal<AngularVelocity> pitchRate;
    private final StatusSignal<AngularVelocity> rollRate;

    // Aceleraciones
    private final StatusSignal<LinearAcceleration> accelX;
    private final StatusSignal<LinearAcceleration> accelY;
    private final StatusSignal<LinearAcceleration> accelZ;

    // -----------------------------------------------------------------------
    // CONSTRUCTOR
    // -----------------------------------------------------------------------

    public GyroIOHardware() {
        this.pigeon = new Pigeon2(SwerveConfig.PIGEON, new CANBus("cleopatra"));

        // Captura StatusSignals.
        yaw      = pigeon.getYaw();
        pitch    = pigeon.getPitch();
        roll     = pigeon.getRoll();
        yawRate  = pigeon.getAngularVelocityZWorld();
        pitchRate = pigeon.getAngularVelocityYWorld();
        rollRate  = pigeon.getAngularVelocityXWorld();
        accelX   = pigeon.getAccelerationX();
        accelY   = pigeon.getAccelerationY();
        accelZ   = pigeon.getAccelerationZ();

        // 100 Hz para orientación y velocidades angulares (el swerve las necesita rápido).
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                yaw, pitch, roll,
                yawRate, pitchRate, rollRate
        );

        // 50 Hz para aceleraciones (se usan para telemetría, no para control).
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                accelX, accelY, accelZ
        );

        // Silencia señales CAN no configuradas explícitamente.
        pigeon.optimizeBusUtilization();

        // Registra todas las señales con AdvantageKit.
        PhoenixUtil.registerSignals(
                false,
                yaw, pitch, roll,
                yawRate, pitchRate, rollRate,
                accelX, accelY, accelZ
        );
    }

    // -----------------------------------------------------------------------
    // UPDATE INPUTS (AdvantageKit)
    // -----------------------------------------------------------------------

    /**
     * Rellena el record de inputs con los valores actuales del Pigeon2.
     * {@code gyroConnected} es {@code true} sólo si todas las señales
     * llegaron sin errores de CAN.
     */
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.gyroData = new GyroIOData(
                // gyroConnected
                BaseStatusSignal.isAllGood(
                        yaw, pitch, roll,
                        yawRate, pitchRate, rollRate,
                        accelX, accelY, accelZ),

                // Orientación en grados
                yaw.getValueAsDouble(),
                pitch.getValueAsDouble(),
                roll.getValueAsDouble(),

                // Velocidades angulares en rad/s
                yawRate.getValueAsDouble(),
                pitchRate.getValueAsDouble(),
                rollRate.getValueAsDouble(),

                // Aceleraciones en m/s² (magnitude descarta la unidad)
                accelX.getValue().magnitude(),
                accelY.getValue().magnitude(),
                accelZ.getValue().magnitude()
        );
    }

    // -----------------------------------------------------------------------
    // COMANDOS
    // -----------------------------------------------------------------------

    /**
     * Resetea el yaw del Pigeon2 al ángulo indicado.
     *
     * @param yawDeg ángulo destino en grados (0 = azul, 180 = rojo).
     */
    @Override
    public void setYaw(double yawDeg) {
        pigeon.setYaw(yawDeg);
    }
}