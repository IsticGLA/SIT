package istic.gla.goupb.nivimoju.drone.engine;


import entity.Drone;
import entity.Intervention;
import entity.Path;
import istic.gla.groupb.nivimoju.drone.client.DroneClient;
import istic.gla.groupb.nivimoju.drone.latlong.LatLongConverter;
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
    private final DroneContainer container;

    //map des paths en coordonnées blender
    private Map<Long, Collection<LocalPath>> localPathsByIntervention;
    private Map<Long, Collection<Drone>> dronesByIntervention;
    private Map<String, Drone> droneByLabel;
    private Map<String, LocalPath> affectationByDroneLabel;

    private DroneEngine(){
        client = new DroneClient();
        container = DroneContainer.getInstance();
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
        }
        return instance;
    }

    /**
     *
     * @param intervention l'intervention à préparer
     */
    public void computeForIntervention(Intervention intervention){
        if(intervention == null){
            logger.warn("got compute order for null intervention, stopping");
            return;
        }
        List<LocalPath> computedPaths = computePaths(intervention);
        Collection<Drone> availableDrones = container.getDronesAssignedTo(intervention.getId());
        if(computedPaths.size() != availableDrones.size()){
            logger.error(String.format("inconsistent sizes. Got %d drones for %d paths", availableDrones.size(), computedPaths.size()));
        }

        Iterator<Drone> droneIterator = availableDrones.iterator();
        HashMap<String, LocalPath> orders = new HashMap<>();
        for(LocalPath path : computedPaths){
            if(droneIterator.hasNext()){
                Drone drone = droneIterator.next();
                orders.put(drone.getLabel(), path);
            } else {
                logger.warn("reached the end of available drone but there are more paths. stopping.");
                break;
            }
        }
        sendOrders(orders);
    }

    /**
     * transforme l'ensemble des chemins et zones d'une intervention en une liste de chemins locaux
     * @param intervention
     * @return
     */
    private List<LocalPath> computePaths(Intervention intervention){
        //TODO augmenter avec les calculs a partir de zones
        return transformInLocal(intervention.getWatchPath());
    }

    /**
     * transforme un path en latlong vers un path local
     * @param paths la liste des path a convertir
     * @return la liste après conversion
     */
    private List<LocalPath> transformInLocal(List<Path> paths){
        List<LocalPath> localPaths = new ArrayList<>();
        if(paths == null){
            return localPaths;
        }
        for(Path path : paths){
            logger.debug("converting path " + path.toString());
            LocalPath pathConverted = converter.getLocalPath(path);
            localPaths.add(pathConverted);
            logger.debug("conversion : " + pathConverted);
        }
        return localPaths;
    }

    /**
     * send order to the simulation for all the drones in an intervention
     * @param orders the orders (path by drone label)
     */
    public void sendOrders(HashMap<String, LocalPath> orders){
        if(orders == null){
            logger.warn("cannot send null orders");
            return;
        }
        logger.info(String.format("sending %d orders for intervention", orders.size()));
        for(Map.Entry entry : orders.entrySet()){
            logger.info("sending order for drone " + entry.getKey() + " path : " + entry.getValue());
            client.postPath((String) entry.getKey(), (LocalPath) entry.getValue());
        }
    }



    public static void main(String[] args) throws Exception{
        DroneEngine engine = new DroneEngine();
        List<Drone> drones = new ArrayList<>();
/*        Position piscine = new Position(48.115367,-1.63781);
        Position croisement = new Position(48.11498, -1.63795);
        Position croisement2 = new Position(48.114454, -1.639962);
        Path path = new Path();
        path.addPosition(croisement);
        path.addPosition(croisement2);
        //engine.setPath(path);
        //engine.updateDrone();
        List<Path> paths = new ArrayList<>();
        paths.add(path);
        engine.setPathsForIntervention(1, paths);*/
    }
}
