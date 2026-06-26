package frc.robot.subsystems.swerve;

import java.util.Optional;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.PoseConfidenceTracker;
import frc.robot.subsystems.swerve.commands.Drive;
import frc.robot.utils.CollisionDetector;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

/**
 * Estimador centralizado de pose del robot.
 *
 * Fusiona múltiples fuentes de información para estimar la posición y orientación
 * del robot con precisión:
 * <ul>
 *   <li>Odometría de módulos swerve + giroscopio (base continua)</li>
 *   <li>Visión con AprilTags (Limelight) para correcciones periódicas</li>
 *   <li>Detección de colisiones y skid para ajustar confianza</li>
 * </ul>
 *
 * Proporciona API de alto nivel para comandos de movimiento autónomo y
 * conducción asistida.
 */
public class PoseTracker {
    // Controladores para seguimiento de trayectoria
    private final ProfiledPIDController thetaController;
    private final HolonomicDriveController holonomicController;

    // ====================================================================
    // ESTIMADOR DE POSE Y PUBLICACIÓN
    // ====================================================================

    /** Estimador de pose usando cinemática swerve + mediciones de visión. */
    private final SwerveDrivePoseEstimator poseEstimator;

    /** Publisher de pose a NetworkTables para dashboards externos. */
    private final StructPublisher<Pose2d> posePublisher =
            NetworkTableInstance.getDefault()
                    .getStructTopic("Robot position", Pose2d.struct)
                    .publish();

    // ====================================================================
    // DEPENDENCIAS DEL SUBSISTEMA
    // ====================================================================

    /** Subsistema swerve (módulos y sensores de orientación). */
    private final Swerve swerve;
    
    // ====================================================================
    // UTILIDADES AUXILIARES
    // ====================================================================

    /** Rastrea confianza en odometría (detección de skid, movimiento anómalo). */
    private final PoseConfidenceTracker confidenceTracker = new PoseConfidenceTracker();

    /** Detecta impactos/colisiones que invalidan mediciones de odometría. */
    private final CollisionDetector collisionDetector = new CollisionDetector();

    /** Flag para indicar si la pose inicial fue establecida usando visión. */
    private boolean initialPoseSetFromVision = false;

    /** Flag para habilitar/deshabilitar fusión de mediciones de visión. */
    private boolean visionOn = true;
    /**
     * Crea el estimador de pose del robot.
     *
     * Inicializa el {@link SwerveDrivePoseEstimator} con:
     * <ul>
     *   <li>Cinemática del chasis desde {@link ChassisConstants}</li>
     *   <li>Orientación inicial del swerve</li>
     *   <li>Posiciones iniciales de los módulos</li>
     *   <li>Pose inicial desde {@link AutonomousConstants#initialPose}</li>
     * </ul>
     */
    public PoseTracker(GyroIO io, SwerveModuleIO[] moduleIOs, Alliance alliance, Pose2d initialPose) {
        this.swerve = new Swerve(io, moduleIOs, alliance);

        this.poseEstimator = new SwerveDrivePoseEstimator(
                SwerveConfig.KINEMATICS,
                Rotation2d.fromDegrees(0),
                swerve.getSwerveModulePos(),
                initialPose
        );

        // Inicializa controladores para seguimiento de trayectoria (Choreo)
        this.thetaController = new ProfiledPIDController(
                SwerveConfig.P_Z,
                SwerveConfig.I_Z,
                SwerveConfig.D_Z,
                SwerveConfig.Z_CONTROLER
        );
        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.holonomicController = new HolonomicDriveController(
                new PIDController(
                        SwerveConfig.P_X,
                        SwerveConfig.I_X,
                        SwerveConfig.D_X
                ),
                new PIDController(
                        SwerveConfig.P_Y,
                        SwerveConfig.I_Y,
                        SwerveConfig.D_Y
                ),
                thetaController
        );
    }

    // ====================================================================
    // API PRINCIPAL
    // ====================================================================

    /**
     * Retorna el subsistema swerve asociado.
     *
     * @return Subsistema swerve
     */
    public Swerve getSwerve() {
        return swerve;
    }

    /**
     * Configura el comando por defecto del swerve.
     * 
     * Conecta los inputs del operador con el comando {@link Drive} que maneja
     * la conducción asistida, aiming y otros controles de alto nivel.
     *
     * @param xInput proveedor de velocidad X
     * @param yInput proveedor de velocidad Y
     * @param omegaInput proveedor de velocidad angular
     * @param resetYaw proveedor para resetear giroscopio
     * @param assistX proveedor de flag para asistencia en X
     * @param assistY proveedor de flag para asistencia en Y
     * @param assistTheta proveedor de flag para asistencia angular
     * @param aimEnabled proveedor de flag para aiming automático
     * @param targetPose proveedor de pose objetivo para navegación
     * @param aimPose proveedor de pose calculada para aiming
     */
    public void configureDefaultCommands(
        Supplier<Double> xInput,
        Supplier<Double> yInput,
        Supplier<Double> omegaInput,
        Supplier<Boolean> resetYaw,

        Supplier<Pose2d> aimPose    
    ) {
        swerve.setDefaultCommand(
            new Drive(
                swerve,
                this,

                xInput,
                yInput,
                omegaInput,
                resetYaw,

                aimPose
            )
        
        );
    }

    /**
     * Obtiene la pose estimada actual del robot.
     *
     * @return Pose2d con posición y orientación estimadas
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resetea la odometría a una pose dada.
     *
     * @param pose nueva pose de referencia
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
                swerve.getRotation2d(),
                swerve.getSwerveModulePos(),
                pose
        );
    }

    // ====================================================================
    // VISIÓN (LIMELIGHT / APRILTAGS)
    // ====================================================================

    /**
     * Fuerza un reset de odometría usando una medición de visión.
     * 
     * Obtiene la pose de una cámara Limelight y la usa como nueva referencia.
     * Reporta advertencia si no hay medición disponible.
     *
     * @param limelight identificador de la cámara Limelight
     */
    public void forceVisionReset(String limelight) {

        Optional<PoseEstimate> vision = getVisionEstimate(limelight);

        if (vision.isEmpty()) {
            DriverStation.reportWarning(
                    "No se pudo obtener una estimación de pose de " + limelight + " para resetear odometría.",
                    false
            );
            return;
        }

        PoseEstimate estimate = vision.get();

        Pose2d pose = new Pose2d(estimate.pose.getTranslation(), swerve.getRotation2d());

        resetOdometry(pose);
        initialPoseSetFromVision = true;

        SmartDashboard.putString(
                "Force Reset Pose Source",
                limelight
        );
    }

    /**
     * Deshabilita la fusión de mediciones de visión.
     * 
     * La odometría continuará basándose solo en módulos y giroscopio.
     */
    public void turnOffVision() {
        visionOn = false;
    }

    /**
     * Habilita la fusión de mediciones de visión.
     */
    public void turnOnVision() {
        visionOn = true;
    }

    /**
     * Obtiene una estimación de pose de una cámara Limelight.
     * 
     * Valida que haya tags detectados y que los datos sean válidos antes
     * de retornar la estimación.
     *
     * @param limelight identificador de la cámara
     * @return Optional con la estimación si es válida, vacío si no
     */
    private Optional<PoseEstimate> getVisionEstimate(String limelight) {

        if (!LimelightHelpers.getTV(limelight))
            return Optional.empty();

            LimelightHelpers.SetRobotOrientation(
                    limelight,
                    swerve.getHeading(),
                    swerve.getGyroRate(), swerve.getPitch(),
                    swerve.getPitchRate(), swerve.getRoll(),
                    swerve.getRollRate()
            );

        PoseEstimate estimate;

        estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
        

        
        if (estimate == null || estimate.tagCount == 0)
            return Optional.empty();

        return Optional.of(estimate);
    }

    /**
     * Valida si una medición de visión es confiable para fusión.
     * 
     * Criterios de validación:
     * <ul>
     *   <li>Distancia promedio a tags menor a 5.0 metros</li>
     *   <li>Velocidad angular menor a 250 deg/s</li>
     *   <li>Si ya hay pose inicial: error de posición menor a 2.0 metros</li>
     * </ul>
     *
     * @param estimate estimación de visión a validar
     * @return true si la medición pasa los criterios
     */
    private boolean visionGate(PoseEstimate estimate) {

        if (estimate.avgTagDist > 5.0)
            return false;

        if (Math.abs(swerve.getGyroRate()) > 250)
            return false;

        return true;
    }

    /**
     * Ajusta dinámicamente la desviación estándar de la medición de visión.
     * 
     * Factores considerados:
     * <ul>
     *   <li>Distancia promedio a tags (factor de distancia)</li>
     *   <li>Cantidad de tags detectados (factor de cantidad)</li>
     *   <li>Velocidad angular del robot (factor de giro)</li>
     * </ul>
     *
     * @param estimate estimación de visión
     */
    private void applyDynamicVisionStdDevs(PoseEstimate estimate) {

        double distanceFactor = Math.max(estimate.avgTagDist, 1.0);

        double tagFactor;
        if (estimate.tagCount >= 3) tagFactor = 0.4;
        else if (estimate.tagCount == 2) tagFactor = 0.7;
        else tagFactor = 1.2;

        double gyroFactor = 1.0 + Math.abs(swerve.getGyroRate()) / 500.0;

        double xyStd = 0.03 * distanceFactor * tagFactor * gyroFactor;

        poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStd, xyStd, 999999)
        );
    }

    /**
     * Procesa una medición de visión para fusión o inicialización de pose.
     * 
     * Si es la primera medición válida, usa para inicializar la pose.
     * Posteriormente, valida y fusiona las mediciones según criterios de confianza.
     *
     * @param limelight identificador de la cámara
     */
    private void processVision(String limelight) {

        Optional<PoseEstimate> vision = getVisionEstimate(limelight);

        if (vision.isEmpty())
            return;

        PoseEstimate estimate = vision.get();

        if (!visionGate(estimate))
            return;

        applyDynamicVisionStdDevs(estimate);

        Pose2d pose = new Pose2d(estimate.pose.getTranslation(), swerve.getRotation2d());
        double timestamp = estimate.timestampSeconds;

        if (!initialPoseSetFromVision) {

            resetOdometry(pose);
            initialPoseSetFromVision = true;

            SmartDashboard.putString(
                    "Init Pose Source",
                    limelight
            );
        }

        if (confidenceTracker.isSkidding()) {

            poseEstimator.addVisionMeasurement(
                    pose,
                    timestamp
            );
        }
    }

    /**
     * Aplica crop inteligente a la ventana de visión de la cámara.
     * 
     * Si hay tags detectados, centra el crop en el objetivo más cercano.
     * Si no hay visión, resetea el crop a máximo.
     *
     * @param limelight identificador de la cámara
     */
    private void applySmartCrop(String limelight) {

        if (!LimelightHelpers.getTV(limelight)) {

            LimelightHelpers.setCropWindow(
                    limelight,
                    -1,1,-1,1
            );

            return;
        }

        double tx = LimelightHelpers.getTX(limelight);

        double center = tx / 29.8;

        double width = 0.35;

        double xmin = Math.max(-1, center - width);
        double xmax = Math.min(1, center + width);

        LimelightHelpers.setCropWindow(
                limelight,
                xmin,
                xmax,
                -1,
                1
        );
    }

    // ====================================================================
    // CICLO PERIÓDICO
    // ====================================================================

    /**
     * Actualiza la estimación de pose del robot.
     * 
     * Debe ser llamado periódicamente (típicamente desde {@code Robot.periodic()}).
     * Ejecuta en orden:
     * <ol>
     *   <li>Detección de colisiones para invalidar odometría si es necesario</li>
     *   <li>Actualización de odometría por módulos y giroscopio</li>
     *   <li>Actualización de confianza en odometría (skid detection)</li>
     *   <li>Procesamiento de mediciones de Limelight (inicialización + fusión)</li>
     *   <li>Publicación de pose a dashboards y NetworkTables</li>
     * </ol>
     */
    public void periodic() {
        // 1. Detección de colisiones e impactos
        if (collisionDetector.detectImpact(swerve)) {
            collisionDetector.freeze();
        } else {
            poseEstimator.update(
                    swerve.getRotation2d(),
                    swerve.getSwerveModulePos()
            );
        }

        // 2. Actualización de confianza en odometría
        confidenceTracker.update(swerve);

        // 3. Procesamiento de visión (AprilTags / Limelight)
        if (visionOn) {

            applySmartCrop(SwerveConfig.LIMELIGHT_1);
            applySmartCrop(SwerveConfig.LIMELIGHT_2);

            processVision(SwerveConfig.LIMELIGHT_1);
            processVision(SwerveConfig.LIMELIGHT_2);
        }

        // 4. Publicación a dashboards y NetworkTables
        Pose2d estimatedPose = getPose();
        SmartDashboard.putString("Robot Pose", estimatedPose.toString());
        Logger.recordOutput("Swerve/EstimatedPose", poseEstimator.getEstimatedPosition());
        posePublisher.set(estimatedPose);
    }

    public void periodic(SwerveDriveSimulation swerveSim) {
        // Actualiza la simulación del swerve con la pose estimada para mantenerla sincronizada
        Logger.recordOutput("FieldSimulation/RobotPosition", swerveSim.getSimulatedDriveTrainPose());
    }
}