package frc.robot.subsystems.chaneler.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chaneler.Chaneler;
import frc.robot.subsystems.chaneler.ChanelerIO;

/**
 * Comando para establecer la velocidad del chaneler.
 *
 * Envía continuamente la velocidad objetivo al motor
 * hasta que el comando sea interrumpido o cancelado.
 */
public class SetVelocity extends Command {

    /** Interfaz de hardware del subsistema Chaneler. */
    private final ChanelerIO io;

    /** Velocidad objetivo (RPS). */
    private final double velocityRps;

    /**
     * Crea un comando para mover el chaneler a una velocidad dada.
     *
     * @param velocityRps velocidad objetivo en rotaciones por segundo
     * @param io          interfaz de hardware del chaneler
     * @param chaneler    subsistema Chaneler (para requirements)
     */
    public SetVelocity(double velocityRps, ChanelerIO io, Chaneler chaneler) {
        this.velocityRps = velocityRps;
        this.io = io;
        addRequirements(chaneler);
    }

    @Override
    public void execute() {
        io.setVelocity(velocityRps);
    }

    @Override
    public void end(boolean interrupted) {
        io.stop();
    }
}