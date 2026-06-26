package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;

/**
 * Comando para mover el pivote del intake y controlar los spinners.
 *
 * Posiciona el pivote a un ángulo específico y configura las velocidades
 * de los motores de giro (principal y secundario). El comando finaliza cuando
 * el pivote alcanza su posición y los spinners alcanzan su velocidad objetivo.
 */
public class Move extends Command {
    /** Interfaz de hardware del subsistema Intake. */
    IntakeIO io;
    /** Posición objetivo del pivote (radianes). */
    double angle;
    /** Velocidad objetivo del spinner principal (RPS). */
    double spinVelocity;
    /** Velocidad objetivo del spinner secundario (RPS). */
    double spinVelocitySecondary;

    /**
     * Crea un comando para posicionar el pivote y controlar los spinners.
     *
     * @param angle posición objetivo del pivote (radianes)
     * @param spinVelocity velocidad objetivo del spinner principal (RPS)
     * @param spinVelocitySecondary velocidad objetivo del spinner secundario (RPS)
     * @param controller controlador del subsistema
     * @param io interfaz de hardware del subsistema
     * @param intake subsistema Intake (para requirements)
     */
    public Move(double angle, double spinVelocity, IntakeIO io, Intake intake) {
        this.angle = angle;
        this.spinVelocity = spinVelocity;
        this.io = io;

        addRequirements(intake);
    }

    /**
     * Ejecuta el comando enviando el ángulo y velocidades al controlador.
     */
    @Override
    public void execute() {
        io.setPosition(angle);
        io.setVelocity(spinVelocity);
    }
}