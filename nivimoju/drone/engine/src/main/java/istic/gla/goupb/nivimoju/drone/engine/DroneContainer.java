package istic.gla.goupb.nivimoju.drone.engine;

import dao.DroneDAO;
import entity.Drone;
import entity.Position;
import org.apache.log4j.Logger;

import java.util.*;

/**
 * Aggregation class that store the drones for quick access
 *
 * This class handles all modifications of drones data and is the reference for every read operation
 */
public class DroneContainer {
    Logger logger = Logger.getLogger(DroneContainer.class);
    private static DroneContainer instance;

    private Map<String, Drone> droneByLabel;
    private Map<Long, Collection<Drone>> dronesByIntervention;

    /**
     * Initialise the DroneContainer with data from DB
     */
    private DroneContainer(){
        dronesByIntervention = new HashMap<>();
        dronesByIntervention.put(-1L, new HashSet<Drone>());
        droneByLabel = new HashMap<>();
        logger.info("updating drones from database");
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connect();
        List<Drone> drones = droneDAO.getAll();
        droneDAO.disconnect();
        if(drones != null && drones.size() > 0) {
            logger.info("got " + drones.size() + " drones from database");
            loadDrones(drones);
        } else{
            logger.error("failed to load drones");
        }
    }

    public static synchronized DroneContainer getInstance(){
        if(instance == null){
            instance = new DroneContainer();
        }
        return instance;
    }

    /**
     * charge une liste de drone et prépare les maps interne
     * @param drones la liste de drones à charger
     **/
    protected void loadDrones(List<Drone> drones){
        logger.info("loading drones internally");
        droneByLabel.clear();
        dronesByIntervention.clear();
        for(Drone drone : drones) {
            logger.info("loading drone[" + drone.getLabel() +"]");
            //update des listes interne
            droneByLabel.put(drone.getLabel(), drone);
            long idIntervention = drone.getIdIntervention();
            if (dronesByIntervention.get(idIntervention) == null) {
                dronesByIntervention.put(idIntervention, new HashSet<Drone>());
            }
            dronesByIntervention.get(idIntervention).add(drone);
        }
        logger.info("loaded " + droneByLabel.size() + " drones : " + droneByLabel.keySet());
    }

    protected Map<String, Drone> getDroneByLabel(){
        return droneByLabel;
    }

    protected Map<Long, Collection<Drone>> getDronesByIntervention(){
        return dronesByIntervention;
    }

    protected static void destroy(){
        instance = null;
    }

    /**
     * Met un jour un drone avec une nouvelle position GPS
      * @param label le label du drone
     * @param position la nouvelle position en latlong
     */
    public void updateDrone(String label, Position position){
        Drone drone = droneByLabel.get(label);
        if(drone != null) {
            drone.setLatitude(position.getLatitude());
            drone.setLongitude(position.getLongitude());
        } else{
            logger.error("We got info for drone [" + label +
                    "] but it does not seem to exist (existing labels : " + droneByLabel.keySet() + ")");
        }
    }

    /**
     * Accède à l'ensemble des drones
     * @return tous les drones
     */
    public Collection<Drone> getDrones(){
        return droneByLabel.values();
    }

    /**
     * Accède à l'ensemble des drones affecté a une intervention
     * @return tous les drones
     */
    public Collection<Drone> getDronesAssignedTo(Long idIntervention){
        Collection<Drone> drones = dronesByIntervention.get(idIntervention);
        return drones == null ? new HashSet<Drone>() : drones;
    }

    /**
     * request a free drone to assign to a intervention
     * @param idInter the id of the intervention
     * @return the drone is one was assigned or null otherwise
     */
    public Drone requestDrone(Long idInter){
        logger.info("drone request for intervention " + idInter);
        Collection<Drone> freeDrones = dronesByIntervention.get(-1L);
        if(freeDrones != null && freeDrones.size() > 0){
            logger.info("there are free drones : " + freeDrones.size());
            Drone drone = freeDrones.iterator().next();
            logger.info("taking drone " + drone.getLabel());
            if(dronesByIntervention.get(idInter) == null){
                dronesByIntervention.put(idInter, new HashSet<Drone>());
            }
            freeDrones.remove(drone);
            drone.setIdIntervention(idInter);
            dronesByIntervention.get(idInter).add(drone);
            return drone;
        } else{
            logger.info("there are no free drones");
            return null;
        }
    }

    /**
     * remove assignment for a drone
     * @param idInter the intervention to remove a drone from
     * @return true is success
     */
    public boolean freeDrone(Long idInter){
        logger.info("drone release for intervention " + idInter);
        Collection<Drone> affectedDrones = dronesByIntervention.get(idInter);
        if(affectedDrones != null && affectedDrones.size() > 0){
            Drone drone = affectedDrones.iterator().next();
            affectedDrones.remove(drone);
            drone.setIdIntervention(-1L);
            dronesByIntervention.get(-1L).add(drone);
            return true;
        } else{
            logger.error("no drone to release");
            return false;
        }
    }

    /**
     * remove assignement for a ginven drone
     * @param droneToFree the drone
     */
    public void freeDrone(Drone droneToFree){
        logger.info("releasing drone " + droneToFree.getLabel() + " from intervention " + droneToFree.getIdIntervention());
        Collection<Drone> affectedDrones = dronesByIntervention.get(droneToFree.getIdIntervention());
        Drone d = getDroneByLabel(droneToFree.getLabel());
        if(d != null){
            affectedDrones.remove(d);
            dronesByIntervention.get(-1L).add(d);
            d.setIdIntervention(-1L);
        }
    }

    /**
     * get a drone by its name
     * @param label the label
     * @return the drone or null
     */
    public Drone getDroneByLabel(String label){
        return droneByLabel.get(label);
    }
}
