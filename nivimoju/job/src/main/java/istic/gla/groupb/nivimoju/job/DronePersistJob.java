package istic.gla.groupb.nivimoju.job;

import istic.gla.groupb.nivimoju.container.DroneContainer;
import istic.gla.groupb.nivimoju.dao.DroneDAO;
import istic.gla.groupb.nivimoju.entity.Drone;
import org.apache.log4j.Logger;
import org.quartz.DisallowConcurrentExecution;
import org.quartz.Job;
import org.quartz.JobExecutionContext;
import org.quartz.JobExecutionException;

/**
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
