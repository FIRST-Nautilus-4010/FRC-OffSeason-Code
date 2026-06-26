package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsistema de conducción swerve del robot.
 *
 * <p>Gestiona los cuatro módulos swerve y convierte velocidades de chasis
 * en comandos para los módulos individuales. La lectura del giroscopio
 * está abstraída detrás de {@link SwerveIO}, lo que permite usar hardware
 * real o simulación sin cambiar este subsistema.
 */
public class Swerve extends SubsystemBase {

    // ====================================================================
    // IO LAYER
    // ====================================================================

    /** Abstracción del giroscopio (Pigeon2 en hardware, sim en simulación). */
    private final GyroIO io;

    /** Inputs cacheados del giroscopio, actualizados cada ciclo en periodic(). */
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    // ====================================================================
    // MÓDULOS DE RUEDA
    // ====================================================================

    /** Módulo swerve delantero izquierdo. */
    private final SwerveModule frontLeft;

    /** Módulo swerve delantero derecho. */
    private final SwerveModule frontRight;

    /** Módulo swerve trasero izquierdo. */
    private final SwerveModule backLeft;

    /** Módulo swerve trasero derecho. */
    private final SwerveModule backRight;

    private final SwerveModuleIOInputsAutoLogged[] moduleInputs = new SwerveModuleIOInputsAutoLogged[4];

    // ====================================================================
    // PUBLICADORES A NETWORKTABLES
    // ====================================================================

    /** Estados medidos actuales de los módulos (para telemetría). */
    private final StructArrayPublisher<SwerveModuleState> swervePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("Detected module states", SwerveModuleState.struct)
                    .publish();

    /** Estados deseados de los módulos (comandos). */
    private final StructArrayPublisher<SwerveModuleState> swerveDesiredStatePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("desiredStates", SwerveModuleState.struct)
                    .publish();

    private final Alliance alliance;

    // ====================================================================
    // CONSTRUCTOR
    // ====================================================================

    /**
     * Crea el subsistema Swerve.
     *
     * @param io       implementación del giroscopio ({@link SwerveIOHardware} en real,
     *                 {@link SwerveIOSim} en simulación)
     * @param alliance alianza actual (afecta el yaw inicial del giroscopio)
     * @param isSimulation indica si se está ejecutando en simulación
     */
    public Swerve(GyroIO io, SwerveModuleIO[] moduleIOs, Alliance alliance) {
        this.io = io;
        this.alliance = alliance;

        for (int i = 0; i < 4; i++) {
            moduleInputs[i] = new SwerveModuleIOInputsAutoLogged();
        }

        frontLeft  = new SwerveModule(moduleIOs[0]);
        frontRight = new SwerveModule(moduleIOs[1]);
        backLeft   = new SwerveModule(moduleIOs[2]);
        backRight  = new SwerveModule(moduleIOs[3]);


        zeroHeading();
    }

    // ====================================================================
    // CICLO PERIÓDICO
    // ====================================================================

    /**
     * Actualiza el subsistema periódicamente.
     *
     * <p>Primero refresca los inputs del giroscopio desde el IO layer,
     * luego publica telemetría a NetworkTables y SmartDashboard.
     */
    @Override
    public void periodic() {
        // Actualiza los inputs del IO layer — SIEMPRE primero en periodic().
        io.updateInputs(inputs);

        Logger.processInputs("Swerve/Gyro", inputs);
        
        frontLeft.periodic("Front Left");
        frontRight.periodic("Front Right");
        backLeft.periodic("Back Left");
        backRight.periodic("Back Right");

        swervePublisher.set(getSwerveModuleStates());
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    // ====================================================================
    // ESTADOS DE MÓDULO Y CHASIS
    // ====================================================================

    /**
     * Obtiene las posiciones actuales de los cuatro módulos.
     *
     * @return Array de posiciones de módulo [FL, FR, BL, BR]
     */
    public SwerveModulePosition[] getSwerveModulePos() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    /**
     * Obtiene los estados actuales de los cuatro módulos.
     *
     * @return Array de estados de módulo [FL, FR, BL, BR]
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    /**
     * Obtiene la orientación actual del robot como {@link Rotation2d}.
     *
     * @return Rotation2d con el heading actual
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // ====================================================================
    // ACELERACIONES  (leídas desde inputs cacheados)
    // ====================================================================

    /** @return Aceleración en X en m/s² */
    public double getAccelX() {
        return inputs.gyroData.accelX();
    }

    /** @return Aceleración en Y en m/s² */
    public double getAccelY() {
        return inputs.gyroData.accelY();
    }

    /** @return Aceleración en Z en m/s² */
    public double getAccelZ() {
        return inputs.gyroData.accelZ();
    }

    /**
     * Calcula el módulo total de la aceleración lineal.
     *
     * @return Magnitud de aceleración (m/s²)
     */
    public double getLinearAcceleration() {
        double ax = getAccelX();
        double ay = getAccelY();
        double az = getAccelZ();
        return Math.sqrt(ax * ax + ay * ay + az * az);
    }

    // ====================================================================
    // VELOCIDADES DE CHASIS
    // ====================================================================

    /** @return Velocidad promedio de las cuatro ruedas (m/s) */
    public double getAverageWheelSpeed() {
        SwerveModuleState[] states = getSwerveModuleStates();
        double sum = 0.0;
        for (SwerveModuleState state : states) {
            sum += state.speedMetersPerSecond;
        }
        return sum / states.length;
    }

    /** @return ChassisSpeeds calculado desde los estados de módulo */
    public ChassisSpeeds getChassisSpeed() {
        return SwerveConfig.KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
    }

    /** @return Velocidad angular del chasis (rad/s) */
    public double getChassisAngularSpeed() {
        return getChassisSpeed().omegaRadiansPerSecond;
    }

    /** @return Velocidad lineal en X del chasis (m/s) */
    public double getXChassisSpeed() {
        return getChassisSpeed().vxMetersPerSecond;
    }

    /** @return Velocidad lineal en Y del chasis (m/s) */
    public double getYChassisSpeed() {
        return getChassisSpeed().vyMetersPerSecond;
    }

    /** @return Magnitud de la velocidad lineal del chasis (m/s) */
    public double getChassisSpeedMagnitude() {
        ChassisSpeeds speeds = getChassisSpeed();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    // ====================================================================
    // SENSORES DE ORIENTACIÓN  (leídos desde inputs cacheados)
    // ====================================================================

    /**
     * Resetea el yaw del giroscopio según la alianza:
     * 0° para azul, 180° para roja.
     */
    public void zeroHeading() {
        io.setYaw(alliance == Alliance.Blue ? 0.0 : 180.0);
    }

    /** @return Heading (yaw) actual en grados */
    public double getHeading() {
        return inputs.gyroData.headingDeg();
    }

    /** @return Pitch actual en grados */
    public double getPitch() {
        return inputs.gyroData.pitchDeg();
    }

    /** @return Roll actual en grados */
    public double getRoll() {
        return inputs.gyroData.rollDeg();
    }

    /** @return Velocidad angular de yaw (rad/s) */
    public double getGyroRate() {
        return inputs.gyroData.gyroRateRadPerSec();
    }

    /** @return Velocidad angular de pitch (rad/s) */
    public double getPitchRate() {
        return inputs.gyroData.pitchRateRadPerSec();
    }

    /** @return Velocidad angular de roll (rad/s) */
    public double getRollRate() {
        return inputs.gyroData.rollRateRadPerSec();
    }

    // ====================================================================
    // CONTROL DE MÓDULOS Y CONDUCCIÓN
    // ====================================================================

    /** Detiene todos los módulos swerve. */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Conduce en modo field-relative.
     *
     * @param xSpeed velocidad en X del campo (m/s)
     * @param ySpeed velocidad en Y del campo (m/s)
     * @param rot    velocidad angular (rad/s)
     */
    public void driveFieldRelative(double xSpeed, double ySpeed, double rot) {
        ChassisSpeeds fieldRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d());
        drive(fieldRelativeSpeeds, false);
    }

    /**
     * Conduce en modo field-relative con asistencia de conducción.
     *
     * @param xSpeed       velocidad en X del campo (m/s)
     * @param ySpeed       velocidad en Y del campo (m/s)
     * @param rot          velocidad angular (rad/s)
     * @param targetVector array [ángulo (rad), distancia (m)] para asistencia
     */
    public void driveFieldRelative(double xSpeed, double ySpeed, double rot, double[] targetVector) {
        ChassisSpeeds fieldRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d());
        drive(fieldRelativeSpeeds, targetVector);
    }

    /**
     * Conduce con asistencia automática de alineación lateral.
     *
     * @param speeds       velocidades de chasis (marco del robot)
     * @param targetVector array [ángulo (rad), distancia (m)] del objetivo
     */
    public void drive(ChassisSpeeds speeds, double[] targetVector) {
        double speedsAngle = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
        double angleToTarget = speedsAngle - targetVector[0];

        if (Math.abs(angleToTarget) >= Math.PI / 2) {
            drive(speeds, false);
            return;
        }

        double assistModule = Math.sin(speedsAngle - targetVector[0]) * targetVector[1];

        ChassisSpeeds asistedVector = new ChassisSpeeds(
                speeds.vyMetersPerSecond  * assistModule * SwerveConfig.ASSIST_STRAFE_FACTOR,
                -speeds.vxMetersPerSecond * assistModule * SwerveConfig.ASSIST_STRAFE_FACTOR,
                speeds.omegaRadiansPerSecond
        );

        drive(asistedVector, false);
    }

    /**
     * Conduce el robot con velocidades en el marco del robot.
     *
     * @param speeds                    velocidades de chasis (vx, vy, ω)
     * @param accelerationLimitsDisabled si es true, omite los límites de aceleración
     */
    public void drive(ChassisSpeeds speeds, boolean accelerationLimitsDisabled) {
        SwerveModuleState[] moduleStates =
                SwerveConfig.KINEMATICS.toSwerveModuleStates(speeds);

        setStates(moduleStates, accelerationLimitsDisabled);
        swerveDesiredStatePublisher.set(moduleStates);
    }

    /**
     * Aplica estados deseados a los módulos con desaturación.
     *
     * @param desiredStates array de estados [FL, FR, BL, BR]
     * @param accelerationLimitsDisabled si es true, omite los límites de aceleración
     */
    public void setStates(SwerveModuleState[] desiredStates, boolean accelerationLimitsDisabled) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates,
                SwerveConfig.MAX_VELOCITY_METERS_PER_SECOND);

        double roll  = getRoll();
        double pitch = getPitch();

        frontLeft.setDesiredState(desiredStates[0],  roll, pitch, accelerationLimitsDisabled);
        frontRight.setDesiredState(desiredStates[1], roll, pitch, accelerationLimitsDisabled);
        backLeft.setDesiredState(desiredStates[2],   roll, pitch, accelerationLimitsDisabled);
        backRight.setDesiredState(desiredStates[3],  roll, pitch, accelerationLimitsDisabled);
    }
}