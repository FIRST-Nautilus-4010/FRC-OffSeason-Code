// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.utils.PathPlannerAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.utils.PathPlannerAutoBuilder;
/**
 * Contenedor central de configuración del robot.
 *
 * Gestiona la inicialización de subsistemas, configuración de controles del operador
 * y bindings de comandos. Coordina la comunicación entre los inputs del joystick y
 * las transiciones de estado del robot.
 */
public class RobotContainer {

  /** Controlador Xbox del driver (puerto 0). */
  private final XboxController driverJoystick;

  /** Gestor centralizado de subsistemas y estados del robot. */
  public final SubsystemManager subsystemManager;

  /** Chooser de comandos autónomos para selección en SmartDashboard/Shuffleboard. */
  private final SendableChooser<Command> autoChooser;

  Alliance alliance;

  /**
   * Crea e inicializa el contenedor del robot.
   *
   * Instancia el gestor de subsistemas, configura los bindings de comandos
   * y establece el estado inicial del robot.
   */
  public RobotContainer(boolean isSimulation) {

    this.driverJoystick = new XboxController(0);

    this.alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    this.subsystemManager = new SubsystemManager(alliance, new Pose2d(), isSimulation);

    configureBindings();

    subsystemManager.initialize();

    PathPlannerAutoBuilder.initialize(subsystemManager);

    boolean isCompetition = true;

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> !auto.getName().startsWith("Test")) // Excluye "Test" en competencia
        : stream // Incluye todas las opciones en pruebas

    );

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Inicializa el robot en modo teleoperado.
   * 
   * Establece el estado inicial a TRAVEL para permitir la conducción normal.
   */
  public void initializeTeleOp() {
    subsystemManager.executeState(RobotState.TRAVEL);
  }

  /**
   * Configura los bindings de comandos del operador.
   * 
   * Mapea los inputs del controlador Xbox a transiciones de estado y controles
   * de conducción:
   * <ul>
   *   <li><b>Joystick izquierdo:</b> Velocidades X e Y (TRAVEL)</li>
   *   <li><b>Joystick derecho X:</b> Velocidad angular (TRAVEL)</li>
   *   <li><b>Botón A:</b> Reset del giroscopio</li>
   * </ul>
   * 
   * Los triggers se invierten según la alianza (Red vs Blue) para una
   * conducción intuitiva en ambos lados del campo.
   */
  private void configureBindings() {
    // ====================================================================
    // CONFIGURACIÓN DE ALIANZA Y MULTIPLICADOR DE LADO
    // ====================================================================
    
    final double sideMultiplier;


    if (alliance == Alliance.Red) {
      sideMultiplier = -1;
    } else {
      sideMultiplier = 1;
    }

    
    subsystemManager.configureTravelControls(
        () -> -driverJoystick.getLeftY() * sideMultiplier,
        () -> -driverJoystick.getLeftX() * sideMultiplier,
        () -> -driverJoystick.getRightX(),
        () -> driverJoystick.getAButton()
    );

    final Trigger travelTrigger = new Trigger(() -> driverJoystick.getXButton());
    travelTrigger.onTrue(new InstantCommand(() -> subsystemManager.executeState(RobotState.TRAVEL)));

    final Trigger intakeTrigger = new Trigger(() -> driverJoystick.getLeftTriggerAxis() > 0.5);
    intakeTrigger.onTrue(new InstantCommand(() -> subsystemManager.executeState(RobotState.INTAKE)));
    
    final Trigger shooterTrigger = new Trigger(() -> driverJoystick.getRightTriggerAxis() > 0.5);
    shooterTrigger.onTrue(new InstantCommand(() -> subsystemManager.executeState(RobotState.SHOOT)));
  }  
  

  /**
   * Retorna el comando autónomo a ejecutar.
   * 
   * Proporciona la rutina de autonomía del robot seleccionada en el chooser
   * para la fase autónoma de la competencia. Si no hay rutina seleccionada,
   * retorna un comando vacío.
   *
   * @return Comando autónomo a ejecutar (o comando vacío si no hay selección)
   */
  public Command getAutonomousCommand() {
    Command selectedCommand = autoChooser.getSelected();
    
    // Validación: si no hay comando seleccionado, retorna un comando vacío
    if (selectedCommand == null) {
      System.err.println("[RobotContainer] ADVERTENCIA: No hay rutina autónoma seleccionada.");
      return Commands.none(); // Retorna comando vacío
    }
    
    return selectedCommand;
  }

  /**
   * Actualiza el estado periódico del robot.
   * 
   * Debe ser llamado desde {@code Robot.robotPeriodic()}. Actualiza todos
   * los subsistemas y la estimación de pose del robot. Mantiene sincronizado
   * el AutoChooser en SmartDashboard/Shuffleboard.
   */
  public void periodic() {
    subsystemManager.periodic();
    
    // Mantiene el AutoChooser sincronizado en el dashboard
    // Esto asegura que siempre esté disponible en Shuffleboard
    if (autoChooser != null) {
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }
     
  }

  /**
   * Desactiva el robot y detiene todos los subsistemas.
   * 
   * Debe ser llamado desde {@code Robot.disabledInit()}. Cancela comandos
   * activos y desactiva los sistemas de conducción.
   */
  public void disable() {
    subsystemManager.disable();
  }

}
