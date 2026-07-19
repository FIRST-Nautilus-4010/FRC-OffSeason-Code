package frc.robot.subsystems.chaneler;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class ChanelerIOHardware implements ChanelerIO {

    // --- Motor ---
    private final TalonFX chanelerMotor;

    // --- Configuración ---
    private final TalonFXConfiguration chanelerConfig;

    // --- Requests ---
    private final MotionMagicVelocityVoltage velocityRequest;
    private final VoltageOut voltageRequest;
    private final DutyCycleOut dutyCycleRequest;

    // --- Status Signals ---
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Temperature> temperature;

    public ChanelerIOHardware() {
        this.chanelerConfig = new TalonFXConfiguration();

        configureMotor();
        configureGains();
        configureMotionMagic();

        this.velocityRequest  = new MotionMagicVelocityVoltage(0.0).withSlot(0).withUpdateFreqHz(50);
        this.voltageRequest   = new VoltageOut(0.0).withUpdateFreqHz(50);
        this.dutyCycleRequest = new DutyCycleOut(0.0).withUpdateFreqHz(50);

        this.chanelerMotor = new TalonFX(ChanelerConfig.CHANELER_TALONFX_ID);

        PhoenixUtil.tryUntilOk(5, () -> chanelerMotor.getConfigurator().apply(chanelerConfig));

        // Status signals
        velocity      = chanelerMotor.getVelocity();
        appliedVolts  = chanelerMotor.getMotorVoltage();
        torqueCurrent = chanelerMotor.getTorqueCurrent();
        supplyCurrent = chanelerMotor.getSupplyCurrent();
        temperature   = chanelerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            velocity, appliedVolts, torqueCurrent, supplyCurrent, temperature
        );

        chanelerMotor.optimizeBusUtilization();

        PhoenixUtil.registerSignals(
            false,
            velocity, appliedVolts, torqueCurrent, supplyCurrent, temperature
        );
    }

    // --------------------------------------------------------------------
    // CONFIGURACIÓN
    // --------------------------------------------------------------------

    private void configureMotor() {
        chanelerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        chanelerConfig.CurrentLimits.SupplyCurrentLimit       = 40;
        chanelerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        chanelerConfig.CurrentLimits.StatorCurrentLimit       = 70;
        chanelerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        chanelerConfig.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        chanelerConfig.Feedback.SensorToMechanismRatio = ChanelerConfig.CHANELER_REDUCTION;
    }

    private void configureGains() {
        var slot0 = chanelerConfig.Slot0;
        slot0.kS = ChanelerConfig.VEL_KS;
        slot0.kV = ChanelerConfig.VEL_KV;
        slot0.kA = ChanelerConfig.VEL_KA;
        slot0.kP = ChanelerConfig.VEL_KP;
        slot0.kI = ChanelerConfig.VEL_KI;
        slot0.kD = ChanelerConfig.VEL_KD;
    }

    private void configureMotionMagic() {
        var mm = chanelerConfig.MotionMagic;
        mm.MotionMagicAcceleration = ChanelerConfig.MAGIC_MOTION_VELOCITY_ACCELERATION;
        mm.MotionMagicJerk         = ChanelerConfig.MAGIC_MOTION_VELOCITY_JERK;
    }

    // --------------------------------------------------------------------
    // FUNCIONES
    // --------------------------------------------------------------------

    @Override
    public void setVelocity(double velocityRps) {
        if (Math.abs(velocityRps) > 0.1) {
            chanelerMotor.setControl(velocityRequest.withVelocity(velocityRps));
        } else {
            chanelerMotor.stopMotor();
        }
    }

    @Override
    public void setVoltage(double volts) {
        chanelerMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void runOpenLoop(double output) {
        chanelerMotor.setControl(dutyCycleRequest.withOutput(output));
    }

    @Override
    public void updateInputs(ChanelerIOInputs inputs) {
        inputs.ChanelerData = new ChanelerIOData(
            BaseStatusSignal.isAllGood(
                velocity, appliedVolts, torqueCurrent, supplyCurrent, temperature
            ),
            velocity.getValueAsDouble(),
            appliedVolts.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            temperature.getValueAsDouble()
        );
    }

    @Override
    public void stop() {
        chanelerMotor.stopMotor();
    }
}