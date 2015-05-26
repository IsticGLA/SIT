package istic.gla.groupb.nivimoju.container;

import istic.gla.groupb.nivimoju.dao.DroneDAO;
import istic.gla.groupb.nivimoju.drone.client.DroneClient;
import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Position;
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

    //the main container, used for read and write operations on drones
    private Map<String, Drone> mapDroneByLabel;
    //this map is used for quick read access to drone assigned to an intervention
    private Map<Long, Collection<Drone>> mapDronesByIntervention;

    /**
     * Initialise the DroneContainer with data from DB
     */
    private DroneContainer(){
        mapDronesByIntervention = new HashMap<>();
        mapDroneByLabel = new HashMap<>();
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
     * charge une liste de drone et prepare les maps interne
     * @param drones la liste de drones a charger
     **/
    protected void loadDrones(List<Drone> drones){
        logger.info("loading drones internally");
        mapDroneByLabel.clear();
        for(Drone drone : drones) {
            logger.info("loading drone[" + drone.getLabel() + "]");
            //update des listes interne
            mapDroneByLabel.put(drone.getLabel(), drone);
        }
        buildMapDronesByIntervention();
        logger.info("loaded " + mapDroneByLabel.size() + " drones : " + mapDroneByLabel.keySet());
    }

    private void buildMapDronesByIntervention(){
        logger.debug("building map of drones by interventions");
        mapDronesByIntervention.clear();
        mapDronesByIntervention.put(-1L, new HashSet<Drone>());
        for(Drone drone : mapDroneByLabel.values()){
            long idIntervention = drone.getIdIntervention();
            if (mapDronesByIntervention.get(idIntervention) == null) {
                mapDronesByIntervention.put(idIntervention, new HashSet<Drone>());
            }
            mapDronesByIntervention.get(idIntervention).add(drone);
        }
        StringBuilder builder = new StringBuilder("built map of drones by interventions :\n");
        for(Map.Entry<Long, Collection<Drone>> entry : mapDronesByIntervention.entrySet()){
            builder.append("intervention ").append(entry.getKey()).append(":\n\t");
            for(Drone drone: entry.getValue()){
                builder.append(drone.getLabel()).append(", ");
            }
            builder.append("\n");
        }
        logger.debug(builder.toString());
    }

    protected Map<String, Drone> getMapDroneByLabel(){
        return mapDroneByLabel;
    }

    protected Map<Long, Collection<Drone>> getMapDronesByIntervention(){
        return mapDronesByIntervention;
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
        Drone drone = mapDroneByLabel.get(label);
        if(drone != null) {
            drone.setLatitude(position.getLatitude());
            drone.setLongitude(position.getLongitude());
        } else{
            logger.error("We got info for drone [" + label +
                    "] but it does not seem to exist (existing labels : " + mapDroneByLabel.keySet() + ")");
        }
    }

    /**
     * Accede a l'ensemble des drones
     * @return tous les drones
     */
    public Collection<Drone> getDrones(){
        return mapDroneByLabel.values();
    }

    /**
     * Accede a l'ensemble des drones affecte a une intervention
     * @return tous les drones
     */
    public Collection<Drone> getDronesAssignedTo(Long idIntervention){
        Collection<Drone> drones = mapDronesByIntervention.get(idIntervention);
        return drones == null ? new HashSet<Drone>() : drones;
    }


    /**
     * request a free drone to assign to a intervention
     * @param idInter the id of the intervention
     * @return the drone is one was assigned or null otherwise
     */
    public Drone requestDrone(Long idInter){
        logger.info("drone request for intervention " + idInter);
        for(Drone drone : mapDroneByLabel.values()){
            if(drone.getIdIntervention() == -1L){
                logger.info("taking " + drone.getLabel());
                drone.setIdIntervention(idInter);
                buildMapDronesByIntervention();
                return drone;
            }
        }
        logger.info("there are no free drones");
        return null;
    }

    /**
     * request x free drones to assign to a intervention
     * @param idInter the id of the intervention
     * @param nbDrones number of drone to take
     */
    public void requestDrones(Long idInter, int nbDrones){
        for(int i = 0; i<nbDrones; i++){
            Drone newDrone = requestDrone(idInter);
            if(newDrone == null)
                break;
        }
    }

    /**
     * remove assignment for a drone
     * @param idInter the intervention to remove a drone from
     * @return true is success
     */
    public boolean freeDrone(Long idInter){
        logger.info("drone release for intervention " + idInter);
        Collection<Drone> affectedDrones = mapDronesByIntervention.get(idInter);
        DroneClient client = new DroneClient();
        if(affectedDrones != null && affectedDrones.size() > 0){
            Drone drone = affectedDrones.iterator().next();
            drone.setIdIntervention(-1L);
            client.postStop(drone.getLabel());
            buildMapDronesByIntervention();
            return true;
        } else{
            logger.error("no drone to release");
            return false;
        }
    }

    /**
     * remove assignment for a drone
     * @param idInter the intervention to remove a drone from
     * @param nbDrones number of drone to free
     * @return true is success
     */
    public void freeDrones(Long idInter, int nbDrones){
        for(int i = 0; i<nbDrones; i++){
            freeDrone(idInter);
        }
    }

    /**
     * remove assignement for a given drone
     * @param label the drone
     */
    public void freeDrone(String label){
        if(label != null){
            Drone droneToFree = getDroneByLabel(label);
            if(droneToFree != null){
                logger.info("releasing drone " + label + " from intervention " + droneToFree.getIdIntervention());
                droneToFree.setIdIntervention(-1L);
                DroneClient client = new DroneClient();
                client.postStop(label);
                buildMapDronesByIntervention();
            } else{
                logger.warn(String.format("cannot free drone because label %s was not found in drone container", label));
            }
        }else{
            logger.warn("cannot free drone because label is null");
        }
    }

    /**
     * get a drone by its name
     * @param label the label
     * @return the drone or null
     */
    public Drone getDroneByLabel(String label){
        return mapDroneByLabel.get(label);
    }
}

