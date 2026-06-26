package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;

/**
 * Comando para establecer la velocidad del indexer.
 *
 * Envía continuamente la velocidad objetivo a los motores
 * hasta que el comando sea interrumpido o cancelado.
 */
public class SetVelocity extends Command {

    /** Interfaz de hardware del subsistema Indexer. */
    private final IndexerIO io;

    /** Velocidad objetivo (RPS). */
    private final double velocityRps;

    /**
     * Crea un comando para mover el indexer a una velocidad dada.
     *
     * @param velocityRps velocidad objetivo en rotaciones por segundo
     * @param io          interfaz de hardware del indexer
     * @param indexer     subsistema Indexer (para requirements)
     */
    public SetVelocity(double velocityRps, IndexerIO io, Indexer indexer) {
        this.velocityRps = velocityRps;
        this.io = io;
        addRequirements(indexer);
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