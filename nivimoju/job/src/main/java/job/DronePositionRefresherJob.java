package job;


import container.DroneContainer;
import entity.Position;
import istic.gla.goupb.nivimoju.drone.engine.DroneEngine;
import istic.gla.groupb.nivimoju.drone.client.DroneClient;
import istic.gla.groupb.nivimoju.drone.client.DroneInfo;
import istic.gla.groupb.nivimoju.drone.client.DronesInfos;
import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;
import org.apache.log4j.Logger;
import org.quartz.*;

/**    public boolean assignDrone(Drone drone){

 * Job refreshing the database with positions from simulation
 */
@DisallowConcurrentExecution
public class DronePositionRefresherJob implements Job{
    Logger logger = Logger.getLogger(DronePositionRefresherJob.class);
    private static DroneClient client = new DroneClient();

    @Override
    public void execute(JobExecutionContext jobExecutionContext) throws JobExecutionException {
        logger.trace("starting job to refresh drone positions");
        DroneContainer container = DroneContainer.getInstance();
        DronesInfos infos = client.getDronesInfos();
        if(infos == null){
            logger.warn("could not get drones infos from flask");
        }else {
            logger.trace("got response from flask client for drones : " + infos);
            for (DroneInfo info : infos.getInfos()) {
                if (info.getPosition() != null) {
                    String label = info.getLabel();
                    double x = info.getPosition().getX();
                    double y = info.getPosition().getY();
                    LocalCoordinate local = new LocalCoordinate(x, y);
                    Position dronePosition = DroneEngine.converter.getLatLong(local);
                    container.updateDrone(label, dronePosition);
                } else {
                    logger.error("got no drone position from flask");
                }
            }
        }
        logger.trace("ending job to refresh drone positions");
    }
}
