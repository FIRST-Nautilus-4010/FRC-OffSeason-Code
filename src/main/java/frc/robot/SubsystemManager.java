package frc.robot;

import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.subsystems.chaneler.Chaneler;
import frc.robot.subsystems.chaneler.ChanelerIO;
import frc.robot.subsystems.chaneler.ChanelerIOHardware;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOHardware;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOHardware;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.PoseTracker;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOHardware;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.swerve.commands.Drive;
import frc.robot.utils.TejuinoBoard;

/**
 * Gestor centralizado de subsistemas y estados del robot.
 *
 * Coordina el funcionamiento de todos los subsistemas (swerve, intake, shooter,
 * Chaneler, climber) y administra las transiciones entre estados operacionales
 * (TRAVEL, INTAKE, SHOOT, CLIMB, TEST).
 * 
 * Mantiene el estado actual del robot y gestiona las asistencias de conducción
 * (assist X, Y, theta) así como el aiming automático hacia los objetivos.
 */
public final class SubsystemManager {

    /** Rastreador de pose del robot (odometría + visión). */
    private final PoseTracker poseTracker;

    private final Intake intake;
    private final Shooter shooter;
    private final Chaneler chaneler;
    private final Indexer indexer;

    /** Controlador de la placa Tejuino para LEDs y feedback visual. */
    //private final TejuinoBoard tejuino;

    /** Estado operacional actual del robot. */
    private RobotState robotState = RobotState.TRAVEL;

    /** Supplier para la velocidad X del robot en modo TRAVEL. */
    private Supplier<Double> travelVxSupplier;

    /** Pose calculada para aiming automático. */
    Pose2d aimPose = new Pose2d(0, 0, new Rotation2d(0));
    
    /** Publisher de NetworkTables para la pose de aiming. */
    StructPublisher<Pose2d> aimPosePublisher = 
        NetworkTableInstance.getDefault()
                    .getStructTopic("Aim Pose", Pose2d.struct)
                    .publish();

    Alliance alliance;

    boolean isSimulation;
    SwerveDriveSimulation driveSim;

    /**
     * Crea el gestor de subsistemas e inicializa todos los subsistemas
     * del robot.
     */ 
    public SubsystemManager(Alliance alliance, Pose2d initialPose, boolean isSimulation) {
        final IntakeIO intakeIO;
        final GyroIO gyroIO;
        final SwerveModuleIO[] moduleIOs;
        final ShooterIO shooterIO;
        final ChanelerIO chanelerIO;
        final IndexerIO indexerIO;

        this.isSimulation = isSimulation;

        if (isSimulation){
            
            driveSim = new SwerveDriveSimulation(
                new DriveTrainSimulationConfig(
                    Kilograms.of(SwerveConfig.MASS_KG),
                    Meters.of(SwerveConfig.BUMPER_LENGTH_X),
                    Meters.of(SwerveConfig.BUMPER_WIDTH_Y),
                    Meters.of(SwerveConfig.WHEELBASE),
                    Meters.of(SwerveConfig.TRACKWIDTH),
                    COTS.ofPigeon2(),
                    () -> new SwerveModuleSimulation(
                        new SwerveModuleSimulationConfig(
                            DCMotor.getKrakenX60(1),
                            DCMotor.getKrakenX44(1),
                            SwerveConfig.PWR_RATIO,
                            SwerveConfig.STR_RATIO,
                            Voltage.ofBaseUnits(SwerveConfig.VEL_KS, Volts),
                            Voltage.ofBaseUnits(SwerveConfig.POS_KS, Volts),
                            Meters.of(SwerveConfig.WHEEL_DIAMETER / 2.0),
                            KilogramSquareMeters.of(0.062),
                            SwerveConfig.FRICTION_COF
                        )
                    )   
                ),
                new Pose2d(3.570, 7.427, new Rotation2d(0))
            );
            

            //driveSim = new SwerveDriveSimulation(DriveTrainSimulationConfig.Default(), new Pose2d(3.570, 7.427, new Rotation2d(0)));
            moduleIOs = new SwerveModuleIO[] {
               new SwerveModuleIOSim(driveSim.getModules()[0]),
               new SwerveModuleIOSim(driveSim.getModules()[1]),
               new SwerveModuleIOSim(driveSim.getModules()[2]),
               new SwerveModuleIOSim(driveSim.getModules()[3])
            };

            final ShooterIOSim shooterSim = new ShooterIOSim(driveSim);

            shooterIO = shooterSim;
            intakeIO = new IntakeIOSim(driveSim, shooterSim);
            gyroIO = new GyroIOSim(driveSim);

            SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
            
        } else {
            shooterIO = new ShooterIOHardware();
            intakeIO = new IntakeIOHardware();  
            gyroIO = new GyroIOHardware();

            moduleIOs = new SwerveModuleIO[] {
                new SwerveModuleIOHardware(SwerveConfig.FL_PWR, SwerveConfig.FL_STR, SwerveConfig.FL_ENC, 0),
                new SwerveModuleIOHardware(SwerveConfig.FR_PWR, SwerveConfig.FR_STR, SwerveConfig.FR_ENC, 0),
                new SwerveModuleIOHardware(SwerveConfig.BL_PWR, SwerveConfig.BL_STR, SwerveConfig.BL_ENC, 0),
                new SwerveModuleIOHardware(SwerveConfig.BR_PWR, SwerveConfig.BR_STR, SwerveConfig.BR_ENC, 0)
            };
        }

        chanelerIO = new ChanelerIOHardware();
        indexerIO = new IndexerIOHardware();


        this.shooter = new Shooter(shooterIO);
        this.intake = new Intake(intakeIO);
        this.chaneler = new Chaneler(chanelerIO);
        this.indexer = new Indexer(indexerIO);

        this.alliance = alliance;

        this.poseTracker = new PoseTracker(gyroIO, moduleIOs, alliance, new Pose2d(3.570, 7.427, new Rotation2d(0)));

        //this.tejuino = new TejuinoBoard();
    }

    /**
     * Configura los controles para el estado TRAVEL.
     * 
     * Debe ser llamado desde {@link RobotContainer} después de crear
     * el SubsystemManager. Conecta los inputs del operador con el PoseTracker
     * y los sistemas de asistencia.
     *
     * @param vx velocidad X del robot (m/s), típicamente del joystick izquierdo
     * @param vy velocidad Y del robot (m/s), típicamente del joystick izquierdo
     * @param omega velocidad angular del robot (rad/s), típicamente del joystick derecho
     * @param resetYaw proveedor para resetear el yaw del giroscopio
     */
    public void configureTravelControls(
            Supplier<Double> vx,
            Supplier<Double> vy,
            Supplier<Double> omega,
            Supplier<Boolean> resetYaw
    ) {
        this.travelVxSupplier = vx;

        poseTracker.configureDefaultCommands(
            vx, 
            vy, 
            omega, 
            resetYaw,
            this::calculateAimPose
        );
    }


    /**
     * Retorna el rastreador de pose del robot.
     */
    public PoseTracker getPoseTracker() {
        return poseTracker;
    }

    private Pose2d calculateAimPose() {
        Pose2d pose = poseTracker.getPose();

        var alliance = DriverStation.getAlliance();

        if (alliance.get() == DriverStation.Alliance.Blue) {
            if (pose.getX() <= 4.625) {
                aimPose = new Pose2d(4.625, 4.033, new Rotation2d(180));
            } else if (pose.getY() >= 4.033) {
                aimPose = new Pose2d(2.580, 6.0495, new Rotation2d(180));
            } else{
                aimPose = new Pose2d(2.580, 2.0165, new Rotation2d(0));
            }
        } else {
            if (pose.getX() >= 16.54 - 4.625) {
                aimPose = new Pose2d(16.54 - 4.625, 4.033, new Rotation2d(180));
            } else if (pose.getY() >= 4.033) {
                aimPose = new Pose2d(16.54 - 2.580, 6.0495, new Rotation2d(180));
            } else{
                aimPose = new Pose2d(16.54 - 2.580, 2.0165, new Rotation2d(0));
            }
        }

        return aimPose;
    }

    /**
     * Inicializa el estado del robot a la configuración operacional por defecto.
     * 
     * Debe ser llamado después de configurar los controles. Establece el estado
     * inicial a TRAVEL e inicializa la placa Tejuino.
     */
    public void initialize() {
        executeState(RobotState.TRAVEL);
        //tejuino.init(40);
    }

    /**
     * Desactiva todos los sistemas de conducción y asistencia.
     * 
     * Deshabilita los flags de asistencia y setea los LEDs a púrpura como
     * indicador visual de estado deshabilitado.
     */
    public void disable() {
        Drive.assistX = false;
        Drive.assistY = false;
        Drive.assistTheta = false;
        Drive.aimEnabled = false;
        //tejuino.all_leds_purple(1);
        //tejuino.all_leds_purple(2);
    }

    /**
     * Cambia el estado actual sin lanzar comandos adicionales.
     *
     * @param state nuevo estado del robot
     */
    private void setState(RobotState state) {
        robotState = state;
    }

    /**
     * Ejecuta un cambio explícito de estado.
     *
     * Cancela todos los comandos programados actualmente y luego programa
     * el nuevo estado. Útil para transiciones inmediatas entre estados.
     *
     * @param state nuevo estado a ejecutar
     */
    public void executeState(RobotState state) {
        CommandScheduler.getInstance().cancelAll();
        scheduleState(state);
    }
    
    /**
     * Programa el comportamiento asociado a un estado operacional.
     * <p>
     * Gestiona la transición a cada estado configurando los subsistemas,
     * asistencias y comandos correspondientes:
     * <ul>
     *   <li><b>TRAVEL:</b> Conducción normal con todos los subsistemas inactivos</li>
     *   <li><b>INTAKE:</b> Activación del intake</li>
     *   <li><b>SHOOT:</b> Preparación del shooter con aiming automático habilitado</li>
     *   <li><b>CLIMB:</b> Preparación del subsistema de escalada</li>
     *   <li><b>TEST:</b> Modo de prueba para subsistemas individuales</li>
     * </ul>
     *
     * @param state estado a programar
     */
    public void scheduleState(RobotState state) {
        setState(state);

        switch (state) {
            case TRAVEL:
                // Verifica que los controles estén configurados
                if (travelVxSupplier == null) {
                    throw new IllegalStateException(
                        "Travel controls not configured. Call configureTravelControls() first."
                    );
                }
                
                CommandScheduler.getInstance().schedule(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            Drive.assistX = false;
                            Drive.assistY = false;
                            Drive.assistTheta = false;
                            Drive.aimEnabled = false;
                            Drive.onAimTolerance = false;
                            //tejuino.all_leds_blue(1);
                            //tejuino.all_leds_blue(2);
                        }),
                        intake.stowCommand(),
                        shooter.stopCommand(),
                        chaneler.stopCommand(),
                        indexer.stopCommand()

                    )
                );
                break;
            case INTAKE:
                CommandScheduler.getInstance().schedule(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            Drive.assistX = false;
                            Drive.assistY = false;
                            Drive.assistTheta = false;
                            Drive.aimEnabled = false;
                            Drive.onAimTolerance = false;
                            //tejuino.all_leds_green(1);
                            //tejuino.all_leds_green(2);
                        }),
                        intake.grabCommand(),
                        shooter.stopCommand(),
                        chaneler.stopCommand(),
                        indexer.stopCommand()
                    )
                );
                break;
            case SHOOT:
                CommandScheduler.getInstance().schedule(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            Drive.assistX = false;
                            Drive.assistY = false;
                            Drive.assistTheta = true;
                            Drive.aimEnabled = true;
                            Drive.targetPose = aimPose;
                            Drive.onAimTolerance = false;
                            //tejuino.all_leds_red(1);
                            //tejuino.all_leds_red(2);
                        }),
                        shooter.setVelocityCommand(0).until(() -> Drive.onAimTolerance).andThen(
                            shooter.releaseCommand(calculateAimPose().getTranslation().getDistance(
                                driveSim.getSimulatedDriveTrainPose().getTranslation()
                            )),
                            chaneler.feedCommand(),
                            indexer.feedCommand()
                        )
                    )
                );
                break;
            default:
                // Todos los demás estados se redirigen a TRAVEL.
                setState(RobotState.TRAVEL);
                break;
        }
    }

    /**
     * Actualiza periódicamente el estado del robot.
     * 
     * Debe ser llamado desde {@code Robot.periodic()}. Actualiza la estimación
     * de pose y publica el estado actual a SmartDashboard para debugging.
     */
    public void periodic() {
        if (isSimulation) {
            poseTracker.periodic(driveSim);
            poseTracker.resetOdometry(driveSim.getSimulatedDriveTrainPose());
        }
        poseTracker.periodic();
        SmartDashboard.putString("Robot State", robotState.toString());

        aimPosePublisher.set(aimPose);
    }
}