package istic.gla.goupb.nivimoju.drone.engine;

import dao.DroneDAO;
import entity.Drone;
import org.apache.log4j.Logger;
import org.quartz.DisallowConcurrentExecution;
import org.quartz.Job;
import org.quartz.JobExecutionContext;
import org.quartz.JobExecutionException;

/**    public boolean assignDrone(Drone drone){

 * Job refreshing the database with positions from simulation
 */
@DisallowConcurrentExecution
public class DronePersistJob implements Job{
    Logger logger = Logger.getLogger(DronePersistJob.class);

    @Override
    public void execute(JobExecutionContext jobExecutionContext) throws JobExecutionException {
        logger.trace("starting job to persist drone");
        DroneContainer container = DroneContainer.getInstance();
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connect();
        for(Drone drone : container.getDrones()) {
            drone.updateDate();
            droneDAO.update(drone);
        }
        droneDAO.disconnect();
        logger.trace("ending job to persist drone");
    }
}
