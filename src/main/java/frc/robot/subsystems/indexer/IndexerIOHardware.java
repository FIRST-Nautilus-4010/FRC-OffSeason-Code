package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class IndexerIOHardware implements IndexerIO {

    // --- Motores ---
    private final TalonFX motorLeft;
    private final TalonFX motorRight;

    // --- Configuración ---
    private final TalonFXConfiguration indexerConfig;

    // --- Requests ---
    private final MotionMagicVelocityVoltage velocityRequest;
    private final VoltageOut voltageRequest;
    private final DutyCycleOut dutyCycleRequest;

    // --- Status Signals ---
    private final StatusSignal<AngularVelocity> velocityLeft;
    private final StatusSignal<AngularVelocity> velocityRight;
    private final StatusSignal<Voltage> appliedVoltsLeft;
    private final StatusSignal<Voltage> appliedVoltsRight;
    private final StatusSignal<Current> torqueCurrentLeft;
    private final StatusSignal<Current> torqueCurrentRight;
    private final StatusSignal<Current> supplyCurrentLeft;
    private final StatusSignal<Current> supplyCurrentRight;

    public IndexerIOHardware() {
        this.indexerConfig = new TalonFXConfiguration();

        configureMotors();
        configureGains();
        configureMotionMagic();

        this.velocityRequest  = new MotionMagicVelocityVoltage(0.0).withSlot(0).withUpdateFreqHz(50);
        this.voltageRequest   = new VoltageOut(0.0).withUpdateFreqHz(50);
        this.dutyCycleRequest = new DutyCycleOut(0.0).withUpdateFreqHz(50);

        this.motorLeft  = new TalonFX(IndexerConfig.INDEXER_L_TALONFX_ID);
        this.motorRight = new TalonFX(IndexerConfig.INDEXER_R_TALONFX_ID);

        PhoenixUtil.tryUntilOk(5, () -> motorLeft.getConfigurator().apply(indexerConfig));
        PhoenixUtil.tryUntilOk(5, () -> motorRight.getConfigurator().apply(indexerConfig));

        // Right sigue al left con dirección opuesta
        motorRight.setControl(new Follower(motorLeft.getDeviceID(), MotorAlignmentValue.Opposed));

        // Status signals
        velocityLeft      = motorLeft.getVelocity();
        velocityRight     = motorRight.getVelocity();
        appliedVoltsLeft  = motorLeft.getMotorVoltage();
        appliedVoltsRight = motorRight.getMotorVoltage();
        torqueCurrentLeft  = motorLeft.getTorqueCurrent();
        torqueCurrentRight = motorRight.getTorqueCurrent();
        supplyCurrentLeft  = motorLeft.getSupplyCurrent();
        supplyCurrentRight = motorRight.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            velocityLeft, velocityRight,
            appliedVoltsLeft, appliedVoltsRight,
            torqueCurrentLeft, torqueCurrentRight,
            supplyCurrentLeft, supplyCurrentRight
        );

        motorLeft.optimizeBusUtilization();
        motorRight.optimizeBusUtilization();

        PhoenixUtil.registerSignals(
            false,
            velocityLeft, velocityRight,
            appliedVoltsLeft, appliedVoltsRight,
            torqueCurrentLeft, torqueCurrentRight,
            supplyCurrentLeft, supplyCurrentRight
        );
    }

    // --------------------------------------------------------------------
    // CONFIGURACIÓN
    // --------------------------------------------------------------------

    private void configureMotors() {
        indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerConfig.CurrentLimits.SupplyCurrentLimit       = 40;
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfig.CurrentLimits.StatorCurrentLimit       = 70;
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        indexerConfig.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        indexerConfig.Feedback.SensorToMechanismRatio = IndexerConfig.INDEXER_REDUCTION;
    }

    private void configureGains() {
        var slot0 = indexerConfig.Slot0;
        slot0.kS = IndexerConfig.VEL_KS;
        slot0.kV = IndexerConfig.VEL_KV;
        slot0.kA = IndexerConfig.VEL_KA;
        slot0.kP = IndexerConfig.VEL_KP;
        slot0.kI = IndexerConfig.VEL_KI;
        slot0.kD = IndexerConfig.VEL_KD;
    }

    private void configureMotionMagic() {
        var mm = indexerConfig.MotionMagic;
        mm.MotionMagicAcceleration = IndexerConfig.MAGIC_MOTION_VELOCITY_ACCELERATION;
        mm.MotionMagicJerk         = IndexerConfig.MAGIC_MOTION_VELOCITY_JERK;
    }

    // --------------------------------------------------------------------
    // FUNCIONES
    // --------------------------------------------------------------------

    @Override
    public void setVelocity(double velocityRps) {
        if (Math.abs(velocityRps) > 0.1) {
            motorLeft.setControl(velocityRequest.withVelocity(velocityRps));
        } else {
            motorLeft.stopMotor();
        }
    }

    @Override
    public void setVoltage(double volts) {
        motorLeft.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void runOpenLoop(double output) {
        motorLeft.setControl(dutyCycleRequest.withOutput(output));
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerData = new IndexerIOData(
            BaseStatusSignal.isAllGood(
                velocityLeft, appliedVoltsLeft, torqueCurrentLeft, supplyCurrentLeft
            ),
            BaseStatusSignal.isAllGood(
                velocityRight, appliedVoltsRight, torqueCurrentRight, supplyCurrentRight
            ),
            velocityLeft.getValueAsDouble(),
            velocityRight.getValueAsDouble(),
            appliedVoltsLeft.getValueAsDouble(),
            appliedVoltsRight.getValueAsDouble(),
            torqueCurrentLeft.getValueAsDouble(),
            torqueCurrentRight.getValueAsDouble(),
            supplyCurrentLeft.getValueAsDouble(),
            supplyCurrentRight.getValueAsDouble()
        );
    }

    @Override
    public void stop() {
        motorLeft.stopMotor();
    }
}