package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

/**
 * Encapsula la configuración y el control del intake:
 * <ul>
 *   <li>Motor de giro en modo Motion Magic Velocity</li>
 *   <li>Motor del pivote en modo Motion Magic Expo (posición)</li>
 * </ul>
 *
 * Esta clase:
 * <ul>
 *   <li>Aplica las ganancias de los slots desde {@link IntakeConstants}</li>
 *   <li>Configura los parámetros de Motion Magic para ambos motores</li>
 *   <li>Provee métodos simples para setear velocidad lineal y ángulo</li>
 * </ul>
 */
public class IntakeIOHardware implements IntakeIO {

    // --- Motores físicos ---

    /** Motor de giro. */
    private final TalonFX spinLeftMotor;
    private final TalonFX spinRightMotor;

    /** Motor del pivote. */
    private final TalonFX pivotMotorLeft;
    private final TalonFX pivotMotorRight;

    // --- Configuración Phoenix 6 ---

    /** Configuración del TalonFX de giro. */
    private final TalonFXConfiguration spinConfig;

    /** Configuración del TalonFX del pivote. */
    private final TalonFXConfiguration pivotConfig;

    // --- Demandos (requests) de control ---

    /**
     * Request de control para velocidad del motor de giro.
     * Usa el Slot0 de la configuración.
     */
    private final MotionMagicVelocityVoltage velocityRequest;

    /**
     * Request de control para posición (Motion Magic Expo) del motor del pivote.
     * Usa el Slot0 de la configuración.
     */
    private final MotionMagicVoltage positionRequest;

    private final VoltageOut voltageRequestSpin;
    private final VoltageOut voltageRequestPivot;

    private final DutyCycleOut dutyCycleRequestPivot;
    private final DutyCycleOut dutyCycleRequestSpin;

    // --- StatusSignals para telemetría ---
    private final StatusSignal<Angle> positionPivot;
    private final StatusSignal<Angle> positionSpin;
    private final StatusSignal<AngularVelocity> velocityPivot;
    private final StatusSignal<AngularVelocity> velocitySpin;
    private final StatusSignal<Voltage> appliedVoltsPivotLeft;
    private final StatusSignal<Voltage> appliedVoltsPivotRight;
    private final StatusSignal<Voltage> appliedVoltsSpinLeft;
    private final StatusSignal<Voltage> appliedVoltsSpinRight;
    private final StatusSignal<Current> torqueCurrentPivotLeft;
    private final StatusSignal<Current> torqueCurrentPivotRight;
    private final StatusSignal<Current> torqueCurrentSpinLeft;
    private final StatusSignal<Current> torqueCurrentSpinRight;
    private final StatusSignal<Current> supplyCurrentPivotLeft;
    private final StatusSignal<Current> supplyCurrentPivotRight;
    private final StatusSignal<Current> supplyCurrentSpinLeft;
    private final StatusSignal<Current> supplyCurrentSpinRight;
    private final StatusSignal<Temperature> temperaturePivotLeft;
    private final StatusSignal<Temperature> temperaturePivotRight;
    private final StatusSignal<Temperature> temperatureSpinLeft;
    private final StatusSignal<Temperature> temperatureSpinRight;

    /**
     * Crea un controlador para un intake.
     *
     * @param spinBackMotor   TalonFX usado como spin (giro)
     * @param spinFrontMotor  TalonFX usado como spin (giro)
     * @param pivotMotor TalonFX usado como pivot (pivote)
     */
    public IntakeIOHardware() {
        // Instancia configuraciones vacías que luego llenamos con nuestras constantes.
        this.spinConfig = new TalonFXConfiguration();
        this.pivotConfig = new TalonFXConfiguration();

        // Configura límites de corriente y modo neutral.
        configureMotors();

        // Requests de control iniciales (valor 0, slot 0).
        this.velocityRequest = new MotionMagicVelocityVoltage(0.0).withSlot(0).withUpdateFreqHz(50);
        this.positionRequest = new MotionMagicVoltage(0.0).withSlot(0).withUpdateFreqHz(50);
        this.voltageRequestSpin = new VoltageOut(0.0).withUpdateFreqHz(50);
        this.voltageRequestPivot = new VoltageOut(0.0).withUpdateFreqHz(50);
        this.dutyCycleRequestPivot = new DutyCycleOut(0.0).withUpdateFreqHz(50);
        this.dutyCycleRequestSpin = new DutyCycleOut(0.0).withUpdateFreqHz(50);

        // Configura gains de slots y parámetros de Motion Magic.
        configureSpinGains();
        configurePivotGains();
        configureMotionMagic();

        // Configura soft limits antes de aplicar las configuraciones al hardware.
        configureSoftLimits();

        this.pivotMotorLeft = new TalonFX(IntakeConfig.PIVOT_L_TALONFX_ID);
        this.pivotMotorRight = new TalonFX(IntakeConfig.PIVOT_R_TALONFX_ID);
        this.spinLeftMotor = new TalonFX(IntakeConfig.SPIN_L_TALONFX_ID);
        this.spinRightMotor = new TalonFX(IntakeConfig.SPIN_R_TALONFX_ID);

        PhoenixUtil.tryUntilOk(
            5, 
            () -> this.pivotMotorLeft.getConfigurator().apply(pivotConfig)
        );
        PhoenixUtil.tryUntilOk(
            5, 
            () -> this.pivotMotorRight.getConfigurator().apply(pivotConfig)
        );
        PhoenixUtil.tryUntilOk(
            5, 
            () -> this.spinLeftMotor.getConfigurator().apply(spinConfig)
        );
        PhoenixUtil.tryUntilOk(
            5, 
            () -> this.spinRightMotor.getConfigurator().apply(spinConfig)
        );

        this.pivotMotorLeft.setControl(new Follower(pivotMotorRight.getDeviceID(), MotorAlignmentValue.Opposed));
        this.spinRightMotor.setControl(new Follower(spinLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        positionPivot = pivotMotorRight.getPosition();
        positionSpin = spinLeftMotor.getPosition();
        velocityPivot = pivotMotorRight.getVelocity();
        velocitySpin = spinLeftMotor.getVelocity();
        appliedVoltsPivotLeft = pivotMotorLeft.getMotorVoltage();
        appliedVoltsPivotRight = pivotMotorRight.getMotorVoltage();
        appliedVoltsSpinLeft = spinLeftMotor.getMotorVoltage();
        appliedVoltsSpinRight = spinRightMotor.getMotorVoltage();
        torqueCurrentPivotLeft = pivotMotorLeft.getTorqueCurrent();
        torqueCurrentPivotRight = pivotMotorRight.getTorqueCurrent();
        torqueCurrentSpinLeft = spinLeftMotor.getTorqueCurrent();
        torqueCurrentSpinRight = spinRightMotor.getTorqueCurrent();
        supplyCurrentPivotLeft = pivotMotorLeft.getSupplyCurrent();
        supplyCurrentPivotRight = pivotMotorRight.getSupplyCurrent();
        supplyCurrentSpinLeft = spinLeftMotor.getSupplyCurrent();
        supplyCurrentSpinRight = spinRightMotor.getSupplyCurrent();
        temperaturePivotLeft = pivotMotorLeft.getDeviceTemp();
        temperaturePivotRight = pivotMotorRight.getDeviceTemp();
        temperatureSpinLeft = spinLeftMotor.getDeviceTemp();
        temperatureSpinRight = spinRightMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            positionPivot, positionSpin, velocityPivot, velocitySpin,
            appliedVoltsPivotLeft, appliedVoltsPivotRight, appliedVoltsSpinLeft, appliedVoltsSpinRight,
            supplyCurrentPivotLeft, supplyCurrentPivotRight, supplyCurrentSpinLeft, supplyCurrentSpinRight,
            temperaturePivotLeft, temperaturePivotRight, temperatureSpinLeft, temperatureSpinRight
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
            250,
            torqueCurrentPivotLeft, torqueCurrentPivotRight, torqueCurrentSpinLeft, torqueCurrentSpinRight
        );

        this.spinLeftMotor.optimizeBusUtilization();
        this.spinRightMotor.optimizeBusUtilization();
        this.pivotMotorLeft.optimizeBusUtilization();
        this.pivotMotorRight.optimizeBusUtilization();

        PhoenixUtil.registerSignals(
            false,
            positionPivot, positionSpin, velocityPivot, velocitySpin,
            appliedVoltsPivotLeft, appliedVoltsPivotRight, appliedVoltsSpinLeft,
            appliedVoltsSpinRight, torqueCurrentPivotLeft, torqueCurrentPivotRight,
            torqueCurrentSpinLeft, torqueCurrentSpinRight, supplyCurrentPivotLeft,
            supplyCurrentPivotRight, supplyCurrentSpinLeft, supplyCurrentSpinRight,
            temperaturePivotLeft, temperaturePivotRight, temperatureSpinLeft, temperatureSpinRight
        );
    }
    // --------------------------------------------------------------------
    // CONFIGURACIÓN
    // --------------------------------------------------------------------
    
    /**
     * Configura los límites de corriente y el modo neutral de ambos motores.
     */
    private void configureMotors() {
        //spinFrontMotor.setControl(new Follower(spinBackMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        spinConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        spinConfig.CurrentLimits.SupplyCurrentLimit = 20;
        spinConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        spinConfig.CurrentLimits.StatorCurrentLimit = 40;
        spinConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        spinConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        spinConfig.Feedback.SensorToMechanismRatio = IntakeConfig.SPIN_REDUCTION;

        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 20;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.Feedback.SensorToMechanismRatio = IntakeConfig.PIVOT_REDUCTION;
    }

    /**
     * Configura las ganancias del slot 0 del motor de giro (velocidad).
     * <p>
     * Valores tomados de {@link IntakeConstants}:
     * kS, kV, kA, kP, kI, kD.
     */
    private void configureSpinGains() {
        var slot0 = spinConfig.Slot0;
        slot0.kS = IntakeConfig.VEL_KS;
        slot0.kV = IntakeConfig.VEL_KV;
        slot0.kA = IntakeConfig.VEL_KA;
        slot0.kP = IntakeConfig.VEL_KP;
        slot0.kI = IntakeConfig.VEL_KI;
        slot0.kD = IntakeConfig.VEL_KD;
    }

    /**
     * Configura las ganancias del slot 0 del motor del pivote (posición).
     * <p>
     * Valores tomados de {@link IntakeConstants}:
     * kG, kS, kV, kA, kP, kI, kD.
     */
    private void configurePivotGains() {
        var slot0 = pivotConfig.Slot0;
        slot0.kG = IntakeConfig.POS_KG;
        slot0.kS = IntakeConfig.POS_KS;
        slot0.kV = IntakeConfig.POS_KV;
        slot0.kA = IntakeConfig.POS_KA;
        slot0.kP = IntakeConfig.POS_KP;
        slot0.kI = IntakeConfig.POS_KI;
        slot0.kD = IntakeConfig.POS_KD;
        slot0.GravityType = GravityTypeValue.Elevator_Static;
    }

    /**
     * Configura los parámetros de Motion Magic para el pivote.
     * <ul>
     *   <li>Pivote: vel. crucero, aceleración, jerk y parámetros Expo</li>
     * </ul>
     */
    private void configureMotionMagic() {
        // Motion Magic Expo en el motor del pivote.
        var pivotMM = pivotConfig.MotionMagic;
        pivotMM.MotionMagicCruiseVelocity = IntakeConfig.MAGIC_MOTION_VELOCITY_STR;
        pivotMM.MotionMagicAcceleration = IntakeConfig.MAGIC_MOTION_ACCELERATION_STR;
        pivotMM.MotionMagicJerk = IntakeConfig.MAGIC_MOTION_JERK_STR;
        pivotMM.MotionMagicExpo_kV = IntakeConfig.MAGIC_MOTION_EXPO_KV_STR;
        pivotMM.MotionMagicExpo_kA = IntakeConfig.MAGIC_MOTION_EXPO_KA_STR;

        // Motion Magic en el motor de giro.
        var spinMM = spinConfig.MotionMagic;
        spinMM.MotionMagicAcceleration = IntakeConfig.MAGIC_MOTION_VELOCITY_ACCELERATION_STR;
        spinMM.MotionMagicJerk = IntakeConfig.MAGIC_MOTION_VELOCITY_JERK_STR;
    }

    /**
     * Configura los soft limits del pivote.
     */
    private void configureSoftLimits() {
        // Configura los soft limits del pivote.
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConfig.PIVOT_SOFT_LIMIT_FORWARD;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConfig.PIVOT_SOFT_LIMIT_REVERSE;
    }

    // --------------------------------------------------------------------
    // FUNCIONES
    // --------------------------------------------------------------------

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotorRight.setControl(voltageRequestPivot.withOutput(voltage));
        
    }

    @Override
    public void setSpinVoltage(double voltage) {
        spinLeftMotor.setControl(voltageRequestSpin.withOutput(voltage));
    }

    @Override
    public void runOpenLoopPivot(double output) {
        pivotMotorRight.setControl(dutyCycleRequestPivot.withOutput(output));
    }

    @Override
    public void runOpenLoopSpin(double output) {
        spinLeftMotor.setControl(dutyCycleRequestSpin.withOutput(output));
    }

    /**
     * Establece la velocidad lineal deseada del intake.
     *
     * @param velocityRps velocidad objetivo en rotaciones por segundo.
     */
    @Override
    public void setVelocity(double radsPerSecond) {
        if (Math.abs(radsPerSecond) > 0.01) {
            spinLeftMotor.setControl(velocityRequest.withVelocity(Units.radiansToRotations(radsPerSecond)));
        } else {
            spinLeftMotor.stopMotor();
        }
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeData =
            new IntakeIOData(
                BaseStatusSignal.isAllGood(positionPivot, positionSpin, velocityPivot, velocitySpin) &&
                BaseStatusSignal.isAllGood(appliedVoltsPivotLeft, appliedVoltsPivotRight, appliedVoltsSpinLeft, appliedVoltsSpinRight) &&
                BaseStatusSignal.isAllGood(torqueCurrentPivotLeft, torqueCurrentPivotRight, torqueCurrentSpinLeft, torqueCurrentSpinRight) &&
                BaseStatusSignal.isAllGood(supplyCurrentPivotLeft, supplyCurrentPivotRight, supplyCurrentSpinLeft, supplyCurrentSpinRight) &&
                BaseStatusSignal.isAllGood(temperaturePivotLeft, temperaturePivotRight, temperatureSpinLeft, temperatureSpinRight),
                Units.rotationsToRadians(positionPivot.getValueAsDouble()),
                Units.rotationsToRadians(velocityPivot.getValueAsDouble()),
                Units.rotationsToRadians(positionSpin.getValueAsDouble()),
                Units.rotationsToRadians(velocitySpin.getValueAsDouble()),
                appliedVoltsPivotLeft.getValueAsDouble(),
                appliedVoltsPivotRight.getValueAsDouble(),
                appliedVoltsSpinLeft.getValueAsDouble(),
                appliedVoltsSpinRight.getValueAsDouble(),
                torqueCurrentPivotLeft.getValueAsDouble(),
                torqueCurrentPivotRight.getValueAsDouble(),
                torqueCurrentSpinLeft.getValueAsDouble(),
                torqueCurrentSpinRight.getValueAsDouble(),
                supplyCurrentPivotLeft.getValueAsDouble(),
                supplyCurrentPivotRight.getValueAsDouble(),
                supplyCurrentSpinLeft.getValueAsDouble(),
                supplyCurrentSpinRight.getValueAsDouble(),
                temperaturePivotLeft.getValueAsDouble(),
                temperaturePivotRight.getValueAsDouble(),
                temperatureSpinLeft.getValueAsDouble(),
                temperatureSpinRight.getValueAsDouble()
            );
    }

    /**
     * Establece el ángulo deseado del módulo.
     *
     * @param angleRad ángulo objetivo en radianes.
     *                 Se convierte a rotaciones del motor de giro usando
     *                 {@link IntakeConstants#ROT_2_RAD}.
     */
    @Override
    public void setPosition(double desPositionRad) {
        if (Math.abs(desPositionRad - positionPivot.getValueAsDouble()) > Math.toRadians(0.1)){
            pivotMotorRight.setControl(positionRequest.withPosition(Units.radiansToRotations(desPositionRad)));
        } else {
            pivotMotorRight.stopMotor();
        }
    }

    @Override
    public void stop() {
        pivotMotorRight.stopMotor();
        spinLeftMotor.stopMotor();
    }
}