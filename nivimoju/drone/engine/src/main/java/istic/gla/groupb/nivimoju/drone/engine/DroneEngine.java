package istic.gla.groupb.nivimoju.drone.engine;

import istic.gla.groupb.nivimoju.container.DroneContainer;
import istic.gla.groupb.nivimoju.entity.*;
import istic.gla.groupb.nivimoju.drone.client.DroneClient;
import istic.gla.groupb.nivimoju.drone.latlong.LatLongConverter;
import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;
import istic.gla.groupb.nivimoju.drone.latlong.LocalPath;
import org.apache.log4j.Logger;
import java.awt.*;
import java.util.*;
import java.util.List;

public class DroneEngine{
    private static final Logger logger = Logger.getLogger(DroneEngine.class);
    public static final LatLongConverter converter =
            new LatLongConverter(48.1222, -1.6428, 48.1119, -1.6337, 720, 1200);
    private static DroneEngine instance;
    private static final double DRONE_SCAN_WIDTH = 5;

    //client vers la simulation
    private DroneClient client;
    private final DroneContainer container;


    private DroneEngine(){
        client = new DroneClient();
        container = DroneContainer.getInstance();
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
     * Compute orders for intervention and send them
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
     * @param intervention l'intervention
     * @return la liste complete des chemins pour les drones
     */
    private List<LocalPath> computePaths(Intervention intervention){
        //TODO augmenter avec les calculs a partir de zones
        List<LocalPath> paths = transformInLocal(intervention.getWatchPath());
        paths.addAll(getPathsForScans(intervention.getWatchArea(), DRONE_SCAN_WIDTH));
        return paths;
    }

    /**
     * transforme un path en latlong vers unsendOrders path local
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
            pathConverted.setTakePictures(true);
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

    public List<LocalPath> getPathsForScans(List<Area> wathAreas, double scanWidth){
        List<LocalPath> paths = new ArrayList<>();
        for(Area area : wathAreas){
            paths.add(getPathForScan(area.getPositions(), scanWidth));
        }
        return paths;
    }

    public LocalPath getPathForScan(List<Position> wathArea, double scanWidth){
        List<LocalCoordinate> localWatchArea = converter.getLocal(wathArea);
        double x0 = Double.MAX_EXPONENT, y0 = Double.MAX_EXPONENT;
        double xMax = Double.MIN_EXPONENT, yMax = Double.MIN_EXPONENT;
        for(LocalCoordinate coord : localWatchArea){
            x0 = Math.min(x0, coord.getX());
            y0 = Math.min(y0, coord.getY());
            xMax = Math.max(xMax, coord.getX());
            yMax = Math.max(yMax, coord.getY());
        }
        Polygon poly = getPolygon(x0, y0, localWatchArea);
        int sizeX = (int) ((xMax - x0) / scanWidth);
        int sizeY = (int) ((yMax - y0) / scanWidth);

        //construit la map de nodes
        logger.info("building node map");
        Node[][] map = new Node[sizeX][sizeY];
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < sizeX; i++) {
            builder.append("\n");
            for (int j = 0; j < sizeY; j++) {
                Node node = new Node();
                LocalCoordinate positionNode = new LocalCoordinate(x0 + i * scanWidth, y0 + j * scanWidth, 30);
                if(poly.contains(positionNode.getX() - x0, positionNode.getY() - y0)){
                    node.setToScan(true);
                    builder.append("x ");
                } else{
                    builder.append("- ");
                }
                node.setObstacle(false);
                node.setVisited(false);
                node.setPosition(positionNode);
                map[i][j] = node;
            }
        }
        if(sizeX > 0 && sizeY > 0) {
            logger.debug("built map : topleft : " + map[0][0].getPosition()
                    + "\nbotright : " + map[sizeX - 1][sizeY - 1].getPosition()
                    + builder.toString());
        }
        return getPathFromMap(map);
    }

    /**
     * get a path (list of point) from a map, passing by each point needing scan
     * @param map the map
     * @return a path
     */
    private LocalPath getPathFromMap(Node[][] map){
        LocalPath path = new LocalPath();
        path.setClosed(false);
        List<LocalCoordinate> positions = new ArrayList<>();
        StringBuilder builder = new StringBuilder("path : ");
        for (int i = 0; i < map.length; i++) {
            builder.append("\n");
            if(i%2 == 0){
                for (int j = 0; j < map[i].length; j++) {
                    if(map[i][j].isToScan()) {
                        positions.add(map[i][j].getPosition());
                        builder.append("(").append(i).append(",").append(j).append(")->");
                    }
                }
            }
            else{
                for (int j = map[i].length -1 ; j >= 0; j--) {
                    if(map[i][j].isToScan()) {
                        positions.add(map[i][j].getPosition());
                        builder.append("(").append(i).append(",").append(j).append(")->");
                    }
                }
            }

        }
        path.setPositions(positions);
        logger.debug(builder.toString());
        return path;
    }


    /**
     * transforme une liste de point dans le repère blender en un polygone décallé
     * @param x0 la position 'origine en x"
     * @param y0 la position 'origine en y"
     * @param points les points du polygone
     * @return le polygone
     */
    protected Polygon getPolygon(double x0, double y0, List<LocalCoordinate> points){
        Polygon res = new Polygon();
        for(LocalCoordinate coord : points){
            int x = (int) (coord.getX() - x0);
            int y = (int) (coord.getY() - y0);
            res.addPoint(x, y);
        }
        return res;
    }

    public static void main(String[] args) throws Exception{
        DroneEngine engine = new DroneEngine();
        Position piscine = new Position(48.115367,-1.63781);
        Position croisement = new Position(48.11498, -1.63795);
        Position croisement2 = new Position(48.114454, -1.639962);
        Path path = new Path();
        path.addPosition(piscine);
        path.addPosition(croisement);
        path.addPosition(croisement2);
        Intervention inter = new Intervention("name", 1, 0, 0);
        ArrayList<Path> paths = new ArrayList<>();
        paths.add(path);
        inter.setWatchPath(paths);

        List<Position> area = new ArrayList<>();
        area.add(piscine);
        area.add(croisement);
        area.add(croisement2);
        Area area1 = new Area();
        area1.setPositions(area);
        List<Area> areas = new ArrayList<>();
        areas.add(area1);
        inter.setWatchArea(areas);

        engine.computeForIntervention(inter);
    }
}
