package istic.gla.goupb.nivimoju.drone.engine;

import org.apache.log4j.Logger;
import org.quartz.*;

/**    public boolean assignDrone(Drone drone){

 * Job refreshing the database with positions from simulation
 */
@DisallowConcurrentExecution
public class DronePositionRefresherJob implements Job{
    Logger logger = Logger.getLogger(DronePositionRefresherJob.class);

    @Override
    public void execute(JobExecutionContext jobExecutionContext) throws JobExecutionException {
        logger.info("starting job to refresh drone positions");
        DroneEngine engine = DroneEngine.getInstance();
        engine.updateDroneInfoFromSimu();
        logger.info("ending job to refresh drone positions");
    }

}
