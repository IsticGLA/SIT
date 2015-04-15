package istic.gla.goupb.nivimoju.drone.engine;

import dao.DroneDAO;
import entity.*;
import istic.gla.groupb.nivimoju.drone.client.DroneClient;
import istic.gla.groupb.nivimoju.drone.client.DroneInfo;
import istic.gla.groupb.nivimoju.drone.client.DronesInfos;
import istic.gla.groupb.nivimoju.drone.latlong.LatLongConverter;
import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;
import istic.gla.groupb.nivimoju.drone.latlong.LocalPath;
import org.apache.log4j.Logger;

import java.util.*;

public class DroneEngine {
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

    public static DroneEngine getInstance(){
        if(instance==null){
            instance = new DroneEngine();
            instance.loadDronesFromDatabase();
        }
        return instance;
    }

    public void setPathsForIntervention(long idIntervention, Collection<Path> paths){
        Collection<LocalPath> localPaths = new ArrayList<>();
        for(Path path : paths){
            localPaths.add(converter.getLocalPath(path));
        }
        localPathsByIntervention.put(idIntervention, localPaths);
        reaffectDronesForIntervention(idIntervention);
    }

    /**
     * sélectionne un drone parmis ceux disponible pour l'intervention pour chaque chemin
     * @param idIntervention l'id de l'intervention
     */
    public void reaffectDronesForIntervention(long idIntervention){
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
            }
        }
    }

    public void sendOrdersForIntervention(long idIntervention){

    }

    /**
     * get positions info from simulation and update database
     */
    public void updateDroneInfoFromSimu(){
        //TODO requete à flask, maj en bdd des positions des drones
        DronesInfos infos = client.getDronesInfos();
        for(DroneInfo info : infos.getInfos()){
            String label = info.getLabel();
            long x = info.getX();
            long y = info.getY();
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
        }
        updateDronesInDatabase();
    }

    private void updateDronesInDatabase(){
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connect();
        for(Drone drone : droneByLabel.values()) {
            drone.updateDate();
            droneDAO.update(drone);
        }
        droneDAO.disconnect();
    }

    public void loadDronesFromDatabase(){
        logger.info("updating drones from database");
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connect();
        List<Drone> drones = droneDAO.getAll();
        droneDAO.disconnect();
        loadDrones(drones);
    }

    public void loadDrones(List<Drone> drones){
        droneByLabel.clear();
        affectationByDroneLabel.clear();
        dronesByIntervention.clear();
        for(Drone drone : drones) {
            //update des listes interne
            droneByLabel.put(drone.getLabel(), drone);
            long idIntervention = drone.getIdIntervention();
            if (dronesByIntervention.get(idIntervention) == null) {
                dronesByIntervention.put(idIntervention, new ArrayList<Drone>());
            }
            dronesByIntervention.get(idIntervention).add(drone);
        }
        logger.info("got " + droneByLabel.size() + " drones from database : " + droneByLabel.keySet());
    }

    public static void main(String[] args) throws Exception{
        DroneEngine engine = new DroneEngine();
/*        Position piscine = new Position(48.115367,-1.63781);
        Position croisement = new Position(48.11498, -1.63795);
        Position croisement2 = new Position(48.114454, -1.639962);
        Path path = new Path();
        path.addPosition(croisement);
        path.addPosition(croisement2);
        List<Path> paths = new ArrayList<>();
        paths.add(path);
        engine.setPathsForIntervention(1, paths);*/
        engine.updateDroneInfoFromSimu();
    }
}
