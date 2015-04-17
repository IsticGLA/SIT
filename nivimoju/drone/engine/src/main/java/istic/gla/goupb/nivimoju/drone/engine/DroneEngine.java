package istic.gla.goupb.nivimoju.drone.engine;

import com.fasterxml.jackson.core.JsonProcessingException;
import dao.DroneDAO;
import entity.Drone;
import entity.Path;
import entity.Position;
import istic.gla.groupb.nivimoju.drone.client.DroneClient;
import istic.gla.groupb.nivimoju.drone.client.DroneInfo;
import istic.gla.groupb.nivimoju.drone.client.DronesInfos;
import istic.gla.groupb.nivimoju.drone.latlong.LatLongConverter;
import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;
import istic.gla.groupb.nivimoju.drone.latlong.LocalPath;
import org.apache.log4j.Logger;

import java.util.*;

public class DroneEngine{
    private static final Logger logger = Logger.getLogger(DroneEngine.class);
    public static final LatLongConverter converter =
            new LatLongConverter(48.1222, -1.6428, 48.1119, -1.6337, 720, 1200);
    private static DroneEngine instance;

    //client vers la simulation
    private DroneClient client;

    //map des paths en coordonnées blender
    private Map<Long, Collection<LocalPath>> localPathsByIntervention;
    private Map<Long, Collection<Drone>> dronesByIntervention;
    private Map<String, Drone> droneByLabel;
    private Map<String, LocalPath> affectationByDroneLabel;

    private DroneEngine(){
        client = new DroneClient();
        localPathsByIntervention = new HashMap<>();
        affectationByDroneLabel = new HashMap<>();
        dronesByIntervention = new HashMap<>();
        droneByLabel = new HashMap<>();
    }

    /**
     * get the DroneEngine instance
     * @return the instance
     */
    public static DroneEngine getInstance(){
        if(instance==null){
            instance = new DroneEngine();
            instance.loadDronesFromDatabase();
        }
        return instance;
    }

    /**
     * Set l'ensemble des chemins à suivre pour une intervention
     * puis réaffecte les drones pour les parcourir et renvois les ordres à la simu
     * @param idIntervention l'id de l'intervention
     * @param paths les chemins demandés
     */
    public void setPathsForIntervention(long idIntervention, Collection<Path> paths){
        Collection<LocalPath> localPaths = new ArrayList<>();
        for(Path path : paths){
            logger.info("converting path " + path.toString());
            LocalPath pathConverted = converter.getLocalPath(path);
            localPaths.add(pathConverted);
            logger.info("conversion : " + pathConverted);
        }
        logger.info("adding " + localPaths.size() + "paths for intervention " + idIntervention);
        localPathsByIntervention.put(idIntervention, localPaths);
        reaffectDronesForIntervention(idIntervention);
        sendOrdersForIntervention(idIntervention);
    }

    /**
     * sélectionne un drone parmis ceux disponible pour l'intervention pour chaque chemin
     * @param idIntervention l'id de l'intervention
     */
    private void reaffectDronesForIntervention(long idIntervention){
        logger.info("reaffecting drone for intervention [" + idIntervention + "]");
        Collection<Drone> availableDrones = dronesByIntervention.get(idIntervention);
        if(availableDrones == null || availableDrones.size() == 0){
            logger.warn("there is no drone available for intervention " + idIntervention);
            return;
        }
        Collection<LocalPath> paths = localPathsByIntervention.get(idIntervention);
        if(paths == null || paths.size() == 0){
            logger.warn("there is no path setted for intervention " + idIntervention);
            return;
        }
        Iterator<Drone> droneIterator = availableDrones.iterator();
        for(LocalPath path : paths){
            if(droneIterator.hasNext()) {
                Drone drone = droneIterator.next();
                affectationByDroneLabel.put(drone.getLabel(), path);
                logger.info("affecting drone " + drone.getLabel() +" to path " + path.toString());
            } else{
                logger.warn("there is not enough drones for intervention : " + availableDrones.size()
                + " drones, " + paths.size() + " paths");
            }
        }
    }

    /**
     * send order to the simulation for all the drones in an intervention
     * @param idIntervention the id of the intervention
     */
    private void sendOrdersForIntervention(long idIntervention){
        logger.info("sending orders for intervention " + idIntervention);
        Collection<Drone> dronesAffected = dronesByIntervention.get(idIntervention);
        if(dronesAffected != null) {
            for (Drone drone : dronesAffected) {
                LocalPath pathForDrone = affectationByDroneLabel.get(drone.getLabel());
                logger.info("sending order for drone " + drone.getLabel() + " path : " + pathForDrone);
                if(pathForDrone != null){
                    try {
                        client.postPath(drone.getLabel(), pathForDrone);
                    } catch (JsonProcessingException e) {
                        logger.error(e);
                    }
                }
            }
        }
    }

    private boolean getDroneInfoFromSimu(){
        DronesInfos infos = client.getDronesInfos();
        if(infos == null){
            logger.warn("could not get infos from flask");
            return false;
        }
        logger.trace("got response from flask client : " + infos);
        for(DroneInfo info : infos.getInfos()){
            if(info.getPosition() != null){
                String label = info.getLabel();
                double x = info.getPosition().getX();
                double y = info.getPosition().getY();
                LocalCoordinate local = new LocalCoordinate(x, y);
                Position dronePosition = converter.getLatLong(local);
                Drone drone = droneByLabel.get(label);
                if(drone != null) {
                    drone.setLatitude(dronePosition.getLatitude());
                    drone.setLongitude(dronePosition.getLongitude());
                } else{
                    logger.error("We got info for drone [" + label +
                            "] but it does not seem to exist (existing labels : " + droneByLabel.keySet() + ")");
                }
            } else{
                logger.error("got no position from flask");
                return false;
            }
        }
        return true;
    }

    /**
     * get positions info from simulation then update database
     */
    public void updateDroneInfoFromSimu(){
        logger.trace("getting positions from simulation");
        boolean gotInfos = getDroneInfoFromSimu();
        if(gotInfos){
            logger.trace("updating db with drones info");
            updateDronesInDatabase();
            logger.trace("done refreshing info for drones");
        }
    }

    /**
     * update the drones information in database (for position)
     */
    private void updateDronesInDatabase(){
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connect();
        for(Drone drone : droneByLabel.values()) {
            drone.updateDate();
            droneDAO.update(drone);
        }
        droneDAO.disconnect();
    }

    /**
     * set all drones from the db
     */
    public void loadDronesFromDatabase(){
        logger.info("updating drones from database");
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connect();
        List<Drone> drones = droneDAO.getAll();
        if(drones != null) {
            logger.info("got " + drones.size() + " drones from database");
        }
        droneDAO.disconnect();
        loadDrones(drones);
    }

    /**
     * associe un drone à une intervention en interne
     * @param drone the drone to assign internally
     * @return vrai si l'opération a réussi
     */
    public boolean assignDrone(Drone drone){
        logger.info("assigning drone");
        if(drone == null || drone.getIdIntervention() == -1) {
            logger.info("cannot assign it");
            return false;
        } else {
            logger.info("drone to assign : " + drone);
            long idIntervention = drone.getIdIntervention();
            if(dronesByIntervention.get(idIntervention) == null){
                dronesByIntervention.put(idIntervention, new ArrayList<Drone>());
            }
            dronesByIntervention.get(idIntervention).add(drone);
            logger.info("old drone in dronesByLabel : " + droneByLabel.get(drone.getLabel()));
            droneByLabel.put(drone.getLabel(), drone);
            logger.info("new drone in dronesByLabel : " + droneByLabel.get(drone.getLabel()));
            affectationByDroneLabel.put(drone.getLabel(), null);
            return true;

        }
    }

    /**
     * retire l'assignetion d'un drone en interne
     * @param drone le drone à unassign
     * @return vrai si l'opération a réussi
     */
    public boolean unasignDrone(final Drone drone){
        if(drone == null){
            logger.warn("cannot unassign it");
            return false;
        } else {
            long idIntervention = drone.getIdIntervention();
            if(dronesByIntervention.get(idIntervention) != null){
                Collection<Drone> dronesOfIntervention = dronesByIntervention.get(idIntervention);
                logger.info("removing drone in list, size "
                        + dronesOfIntervention.size());
                for(Drone droneToTest : dronesOfIntervention){
                    if(droneToTest.getLabel().equals(drone.getLabel())) {
                        //on retire le drone de la liste de drone existante
                        dronesOfIntervention.remove(drone);
                        try {
                            client.postStop(drone.getLabel());
                        } catch (JsonProcessingException e) {
                            e.printStackTrace();
                        }
                    }
                }
                logger.info("removed drone in list, size "
                        + dronesOfIntervention.size());
            }
            droneByLabel.put(drone.getLabel(), drone);
            affectationByDroneLabel.put(drone.getLabel(), null);
            return true;
        }
    }

    /**
     * charge une liste de drone et prépare les maps interne
     * @param drones la liste de drones à charger
     **/
    private void loadDrones(List<Drone> drones){
        logger.info("loading drones internally");
        droneByLabel.clear();
        affectationByDroneLabel.clear();
        dronesByIntervention.clear();
        for(Drone drone : drones) {
            logger.info("loading drone[" + drone.getLabel() +"]");
            //update des listes interne
            droneByLabel.put(drone.getLabel(), drone);
            long idIntervention = drone.getIdIntervention();
            if (dronesByIntervention.get(idIntervention) == null) {
                dronesByIntervention.put(idIntervention, new ArrayList<Drone>());
            }
            dronesByIntervention.get(idIntervention).add(drone);
        }
        logger.info("loaded " + droneByLabel.size() + " drones : " + droneByLabel.keySet());
    }



    public static void main(String[] args) throws Exception{
        DroneEngine engine = new DroneEngine();
        List<Drone> drones = new ArrayList<>();
        drones.add(new Drone("drone_1", 1, 2, -1));
        drones.add(new Drone("drone_2", 1, 2, -1));
        drones.add(new Drone("drone_3", 1, 2, -1));
        drones.add(new Drone("drone_4", 1, 2, -1));
        drones.add(new Drone("drone_5", 1, 2, -1));
        engine.loadDrones(drones);
/*        Position piscine = new Position(48.115367,-1.63781);
        Position croisement = new Position(48.11498, -1.63795);
        Position croisement2 = new Position(48.114454, -1.639962);
        Path path = new Path();
        path.addPosition(croisement);
        path.addPosition(croisement2);
        List<Path> paths = new ArrayList<>();
        paths.add(path);
        engine.setPathsForIntervention(1, paths);*/
        engine.getDroneInfoFromSimu();
        logger.info(drones.get(0).getLatitude() + "; " + drones.get(0).getLongitude());
    }
}
