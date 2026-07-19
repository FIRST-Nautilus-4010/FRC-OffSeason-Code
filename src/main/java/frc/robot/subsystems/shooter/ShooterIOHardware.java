package frc.robot.subsystems.shooter;

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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class ShooterIOHardware implements ShooterIO {

    // --- Motores ---
    private final TalonFX spinMotorLeft;
    private final TalonFX spinMotorRight;

    // --- Configuración ---
    private final TalonFXConfiguration spinConfig;

    // --- Requests ---
    private final MotionMagicVelocityVoltage velocityRequest;
    private final VoltageOut voltageRequest;
    private final DutyCycleOut dutyCycleRequest;

    // --- Status Signals ---
    private final StatusSignal<Angle> positionLeft;
    private final StatusSignal<Angle> positionRight;
    private final StatusSignal<AngularVelocity> velocityLeft;
    private final StatusSignal<AngularVelocity> velocityRight;
    private final StatusSignal<Voltage> appliedVoltsLeft;
    private final StatusSignal<Voltage> appliedVoltsRight;
    private final StatusSignal<Current> supplyCurrentLeft;
    private final StatusSignal<Current> supplyCurrentRight;

    public ShooterIOHardware() {
        this.spinConfig = new TalonFXConfiguration();

        configureMotors();
        configureGains();
        configureMotionMagic();

        this.velocityRequest = new MotionMagicVelocityVoltage(0.0).withSlot(0).withUpdateFreqHz(50);
        this.voltageRequest  = new VoltageOut(0.0).withUpdateFreqHz(50);
        this.dutyCycleRequest = new DutyCycleOut(0.0).withUpdateFreqHz(50);

        this.spinMotorLeft  = new TalonFX(ShooterConfig.SPIN_L_TALONFX_ID);
        this.spinMotorRight = new TalonFX(ShooterConfig.SPIN_R_TALONFX_ID);

        PhoenixUtil.tryUntilOk(5, () -> spinMotorLeft.getConfigurator().apply(spinConfig));
        PhoenixUtil.tryUntilOk(5, () -> spinMotorRight.getConfigurator().apply(spinConfig));

        // Right sigue al left con dirección opuesta
        this.spinMotorRight.setControl(
            new Follower(spinMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed)
        );

        // Status signals (solo del líder para posición/velocidad)
        positionLeft     = spinMotorLeft.getPosition();
        positionRight    = spinMotorRight.getPosition();
        velocityLeft     = spinMotorLeft.getVelocity();
        velocityRight    = spinMotorRight.getVelocity();
        appliedVoltsLeft  = spinMotorLeft.getMotorVoltage();
        appliedVoltsRight = spinMotorRight.getMotorVoltage();
        supplyCurrentLeft  = spinMotorLeft.getSupplyCurrent();
        supplyCurrentRight = spinMotorRight.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            positionLeft, positionRight,
            velocityLeft, velocityRight,
            appliedVoltsLeft, appliedVoltsRight,
            supplyCurrentLeft, supplyCurrentRight
        );

        spinMotorLeft.optimizeBusUtilization();
        spinMotorRight.optimizeBusUtilization();

        PhoenixUtil.registerSignals(
            false,
            positionLeft, positionRight,
            velocityLeft, velocityRight,
            appliedVoltsLeft, appliedVoltsRight,
            supplyCurrentLeft, supplyCurrentRight
        );
    }

    // --------------------------------------------------------------------
    // CONFIGURACIÓN
    // --------------------------------------------------------------------

    private void configureMotors() {
        spinConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        spinConfig.CurrentLimits.SupplyCurrentLimit       = 40;
        spinConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        spinConfig.CurrentLimits.StatorCurrentLimit       = 120;
        spinConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        spinConfig.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        spinConfig.Feedback.SensorToMechanismRatio = ShooterConfig.SPIN_REDUCTION;
    }

    private void configureGains() {
        var slot0 = spinConfig.Slot0;
        slot0.kS = ShooterConfig.VEL_KS;
        slot0.kV = ShooterConfig.VEL_KV;
        slot0.kA = ShooterConfig.VEL_KA;
        slot0.kP = ShooterConfig.VEL_KP;
        slot0.kI = ShooterConfig.VEL_KI;
        slot0.kD = ShooterConfig.VEL_KD;
    }

    private void configureMotionMagic() {
        var mm = spinConfig.MotionMagic;
        mm.MotionMagicAcceleration = ShooterConfig.MAGIC_MOTION_VELOCITY_ACCELERATION_STR;
        mm.MotionMagicJerk         = ShooterConfig.MAGIC_MOTION_VELOCITY_JERK_STR;
    }

    // --------------------------------------------------------------------
    // FUNCIONES
    // --------------------------------------------------------------------

    @Override
    public void setVelocity(double velocityRps) {
        if (Math.abs(velocityRps) > 0.1) {
            spinMotorLeft.setControl(velocityRequest.withVelocity(velocityRps));
        } else {
            spinMotorLeft.stopMotor();
        }
    }

    @Override
    public void setVoltage(double volts) {
        spinMotorLeft.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void runOpenLoop(double output) {
        spinMotorLeft.setControl(dutyCycleRequest.withOutput(output));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterData = new ShooterIOData(
            BaseStatusSignal.isAllGood(
                positionLeft, positionRight,
                velocityLeft, velocityRight,
                appliedVoltsLeft, appliedVoltsRight,
                supplyCurrentLeft, supplyCurrentRight
            ),
            Units.rotationsToRadians(positionLeft.getValueAsDouble()),
            Units.rotationsToRadians(positionRight.getValueAsDouble()),
            Units.rotationsToRadians(velocityLeft.getValueAsDouble()),
            Units.rotationsToRadians(velocityRight.getValueAsDouble()),
            appliedVoltsLeft.getValueAsDouble(),
            appliedVoltsRight.getValueAsDouble(),
            supplyCurrentLeft.getValueAsDouble(),
            supplyCurrentRight.getValueAsDouble()
        );
    }

    @Override
    public void stop() {
        spinMotorLeft.stopMotor();
    }
}