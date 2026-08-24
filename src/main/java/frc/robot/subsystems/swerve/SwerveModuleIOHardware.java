package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

/**
 * Gestiona el hardware completo de un solo módulo swerve:
 * - Motor de tracción (drive) en modo VelocityVoltage
 * - Motor de giro (steer/turning) en modo Motion Magic Expo (posición)
 * - Encoder absoluto del ángulo (CANcoder)
 *
 * Sigue el mismo patrón de IntakeIOHardware:
 * StatusSignals con frecuencias diferenciadas, PhoenixUtil.tryUntilOk
 * para aplicar configuraciones, optimizeBusUtilization y
 * registro de señales con AdvantageKit.
 */
public class SwerveModuleIOHardware implements SwerveModuleIO {

    // -----------------------------------------------------------------------
    // HARDWARE
    // -----------------------------------------------------------------------

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final CANcoder absoluteEncoder;
    private final double absoluteEncoderOffsetRad;

    // -----------------------------------------------------------------------
    // CONFIGURACIÓN PHOENIX 6
    // -----------------------------------------------------------------------

    private final TalonFXConfiguration driveConfig;
    private final TalonFXConfiguration turningConfig;

    // -----------------------------------------------------------------------
    // REQUESTS DE CONTROL
    // -----------------------------------------------------------------------

    /** Velocidad closed-loop (Slot 0). */
    private final VelocityVoltage velocityRequest;

    /** Posición Motion Magic Expo (Slot 0). */
    private final PositionVoltage positionRequest;

    // -----------------------------------------------------------------------
    // STATUS SIGNALS
    // -----------------------------------------------------------------------

    private final StatusSignal<Angle>           drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage>         driveAppliedVolts;
    private final StatusSignal<Current>         driveTorqueCurrent;
    private final StatusSignal<Current>         driveSupplyCurrent;
    private final StatusSignal<Temperature>     driveTemperature;

    private final StatusSignal<Angle>           turningPosition;
    private final StatusSignal<AngularVelocity> turningVelocity;
    private final StatusSignal<Voltage>         turningAppliedVolts;
    private final StatusSignal<Current>         turningTorqueCurrent;
    private final StatusSignal<Current>         turningSupplyCurrent;
    private final StatusSignal<Temperature>     turningTemperature;

    private final StatusSignal<Angle>           absolutePosition;

    // -----------------------------------------------------------------------
    // CONSTRUCTOR
    // -----------------------------------------------------------------------

    public SwerveModuleIOHardware(
            int driveTalonFxId,
            int turningTalonFxId,
            int absoluteEncoderId,
            double absoluteOffsetRad
    ) {
        this.absoluteEncoderOffsetRad = absoluteOffsetRad;

        this.absoluteEncoder = new CANcoder(absoluteEncoderId, new CANBus("cleopatra"));
        this.driveMotor      = new TalonFX(driveTalonFxId,    new CANBus("cleopatra"));
        this.turningMotor    = new TalonFX(turningTalonFxId,  new CANBus("cleopatra"));

        this.driveConfig   = new TalonFXConfiguration();
        this.turningConfig = new TalonFXConfiguration();

        // withUpdateFreqHz(0) — Phoenix no publica la señal de control de forma
        // periódica, sólo cuando se envía el request. Reduce tráfico CAN.
        this.velocityRequest = new VelocityVoltage(0.0).withSlot(0).withUpdateFreqHz(50);
        this.positionRequest = new PositionVoltage(0.0).withSlot(0).withUpdateFreqHz(50);

        configureMotors();
        configureDriveGains();
        configureTurningGains();

        // Reintenta hasta 5 veces si hay falla transitoria de CAN al aplicar config.
        PhoenixUtil.tryUntilOk(
            5, 
            () -> driveMotor.getConfigurator().apply(driveConfig)
        );
        PhoenixUtil.tryUntilOk(
            5, 
            () -> turningMotor.getConfigurator().apply(turningConfig)
        );

        // -- Captura StatusSignals --
        drivePosition      = driveMotor.getPosition();
        driveVelocity      = driveMotor.getVelocity();
        driveAppliedVolts  = driveMotor.getMotorVoltage();
        driveTorqueCurrent = driveMotor.getTorqueCurrent();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();
        driveTemperature   = driveMotor.getDeviceTemp();

        turningPosition      = turningMotor.getPosition();
        turningVelocity      = turningMotor.getVelocity();
        turningAppliedVolts  = turningMotor.getMotorVoltage();
        turningTorqueCurrent = turningMotor.getTorqueCurrent();
        turningSupplyCurrent = turningMotor.getSupplyCurrent();
        turningTemperature   = turningMotor.getDeviceTemp();

        absolutePosition = absoluteEncoder.getAbsolutePosition();

        // 50 Hz — posición, velocidad, voltaje, supply, temperatura, encoder absoluto.
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                drivePosition, driveVelocity, driveAppliedVolts,
                driveSupplyCurrent, driveTemperature,
                turningPosition, turningVelocity, turningAppliedVolts,
                turningSupplyCurrent, turningTemperature,
                absolutePosition
        );

        // 250 Hz — corrientes de torque (control + detección de fallas).
        BaseStatusSignal.setUpdateFrequencyForAll(
                250,
                driveTorqueCurrent, turningTorqueCurrent
        );

        // Silencia señales CAN no configuradas explícitamente.
        driveMotor.optimizeBusUtilization();
        turningMotor.optimizeBusUtilization();
        absoluteEncoder.optimizeBusUtilization();

        // Registra señales con AdvantageKit para logging determinista.
        PhoenixUtil.registerSignals(
                true,
                drivePosition, driveVelocity, driveAppliedVolts,
                driveTorqueCurrent, driveSupplyCurrent, driveTemperature,
                turningPosition, turningVelocity, turningAppliedVolts,
                turningTorqueCurrent, turningSupplyCurrent, turningTemperature,
                absolutePosition
        );

        resetEncoders(absolutePosition.getValueAsDouble() * 2 * Math.PI + absoluteEncoderOffsetRad);
        
        if (driveTalonFxId <= 3) {
            SwerveConfig.orchestra.addInstrument(driveMotor, 0);
            SwerveConfig.orchestra.addInstrument(turningMotor, 1);
        } else {
            SwerveConfig.orchestra.addInstrument(driveMotor, 0);
            SwerveConfig.orchestra.addInstrument(turningMotor, 1);
        }
    }

    // -----------------------------------------------------------------------
    // CONFIGURACIÓN PRIVADA
    // -----------------------------------------------------------------------

    private void configureMotors() {
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable  = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit        = 70;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable  = true;
        driveConfig.CurrentLimits.StatorCurrentLimit        = 120;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent  = 120;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent  = -120;
        driveConfig.MotorOutput.NeutralMode                 = NeutralModeValue.Brake;
        driveConfig.MotorOutput.Inverted                    = InvertedValue.Clockwise_Positive;
        driveConfig.Feedback.SensorToMechanismRatio         = 1.0 / SwerveConfig.ROT_2_M;
        driveConfig.Audio.AllowMusicDurDisable = true;

        turningConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turningConfig.CurrentLimits.SupplyCurrentLimit       = 20;
        turningConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turningConfig.CurrentLimits.StatorCurrentLimit       = 40;
        turningConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        turningConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        turningConfig.MotorOutput.NeutralMode                = NeutralModeValue.Brake;
        turningConfig.MotorOutput.Inverted                   = InvertedValue.CounterClockwise_Positive;
        turningConfig.Feedback.SensorToMechanismRatio        = 1.0 / SwerveConfig.ROT_2_RAD;
        turningConfig.Audio.AllowMusicDurDisable = true;
    }

    private void configureDriveGains() {
        var slot0 = driveConfig.Slot0;
        slot0.kS  = SwerveConfig.VEL_KS;
        slot0.kV  = SwerveConfig.VEL_KV;
        slot0.kA  = SwerveConfig.VEL_KA;
        slot0.kP  = SwerveConfig.VEL_KP;
        slot0.kI  = SwerveConfig.VEL_KI;
        slot0.kD  = SwerveConfig.VEL_KD;
    }

    private void configureTurningGains() {
        var slot0 = turningConfig.Slot0;
        slot0.kG  = SwerveConfig.POS_KG;
        slot0.kS  = SwerveConfig.POS_KS;
        slot0.kV  = SwerveConfig.POS_KV;
        slot0.kA  = SwerveConfig.POS_KA;
        slot0.kP  = SwerveConfig.POS_KP;
        slot0.kI  = SwerveConfig.POS_KI;
        slot0.kD  = SwerveConfig.POS_KD;
    }

    // -----------------------------------------------------------------------
    // RESET DE ENCODERS
    // -----------------------------------------------------------------------

    @Override
    public final void resetEncoders(double absoluteEncoderPosition) {
        turningMotor.setPosition(absoluteEncoderPosition);
        driveMotor.setPosition(0.0);
    }

    // -----------------------------------------------------------------------
    // UPDATE INPUTS (AdvantageKit)
    // -----------------------------------------------------------------------

    /**
     * Rellena el record de inputs con los valores actuales de los StatusSignals.
     * driveConnected / turningConnected son true sólo si todas las señales
     * del grupo llegaron sin errores de CAN.
     */
    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.moduleData = new SwerveModuleIOData(

                // driveConnected
                BaseStatusSignal.isAllGood(
                        drivePosition, driveVelocity, driveAppliedVolts,
                        driveTorqueCurrent, driveSupplyCurrent, driveTemperature),

                // turningConnected (incluye encoder absoluto)
                BaseStatusSignal.isAllGood(
                        turningPosition, turningVelocity, turningAppliedVolts,
                        turningTorqueCurrent, turningSupplyCurrent, turningTemperature,
                        absolutePosition),

                // Drive — metros y m/s
                drivePosition.getValueAsDouble(),
                driveVelocity.getValueAsDouble(),

                // Turning — radianes
                turningPosition.getValueAsDouble(),
                turningVelocity.getValueAsDouble(),

                // Encoder absoluto en radianes (offset + normalizado)
                absolutePosition.getValueAsDouble() * 2 * Math.PI + absoluteEncoderOffsetRad,

                // Voltajes
                driveAppliedVolts.getValueAsDouble(),
                turningAppliedVolts.getValueAsDouble(),

                // Corriente torque
                driveTorqueCurrent.getValueAsDouble(),
                turningTorqueCurrent.getValueAsDouble(),

                // Corriente supply
                driveSupplyCurrent.getValueAsDouble(),
                turningSupplyCurrent.getValueAsDouble(),

                // Temperatura
                driveTemperature.getValueAsDouble(),
                turningTemperature.getValueAsDouble()
        );
    }

    // -----------------------------------------------------------------------
    // COMANDOS DE CONTROL
    // -----------------------------------------------------------------------

    @Override
    public void setVelocity(double velocityMps) {
        driveMotor.setControl(velocityRequest.withVelocity(velocityMps));
    }

    @Override
    public void setAngle(double angleRad) {
        double currentAngle = turningPosition.getValueAsDouble();
        double target = angleRad;

        double error = target - currentAngle;

        error = Math.IEEEremainder(error, 2 * Math.PI);

        double nearestTarget = currentAngle + error;

        turningMotor.setControl(positionRequest.withPosition(nearestTarget));
    }


    @Override
    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }
}