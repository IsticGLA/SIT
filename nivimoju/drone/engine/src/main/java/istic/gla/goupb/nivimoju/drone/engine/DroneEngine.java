package istic.gla.goupb.nivimoju.drone.engine;

import entity.Path;
import entity.Position;
import istic.gla.groupb.nivimoju.drone.client.DroneClient;
import istic.gla.groupb.nivimoju.drone.latlong.LatLongConverter;
import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;
import istic.gla.groupb.nivimoju.drone.latlong.LocalPath;
import org.apache.log4j.Logger;

import java.util.ArrayList;
import java.util.List;

public class DroneEngine {
    private static final Logger logger = Logger.getLogger(DroneEngine.class);
    public static final LatLongConverter converter =
            new LatLongConverter(48.1222, -1.6428, 48.1119, -1.6337, 720, 1200);
    private static DroneEngine instance;

    private DroneClient client;
    private Path path;
    private LocalPath localPath;

    private DroneEngine(){
        client = new DroneClient();
    }

    public static DroneEngine getInstance(){
        if(instance==null){
            instance = new DroneEngine();
        }
        return instance;
    }

    public void setPath(Path path){
        this.path = path;
        this.localPath = new LocalPath();
        localPath.setClosed(path.isClosed());
        List<LocalCoordinate> localCoordinates = new ArrayList<>();
        for(Position latLong : path.getPositions()){
            try{
                LocalCoordinate coord = converter.getLocal(latLong);
                coord.setZ(20);
                localCoordinates.add(coord);
            } catch (IllegalArgumentException e){
                logger.error("could not transfer " + latLong + " to local coordinates");
            }
        }
        localPath.setPositions(localCoordinates);
    }

    private void updateDrone(){
        try {
            client.postPath(localPath);
        }
        catch (Exception e){
            logger.error("failed to update drone", e);
        }
    }

    public static void main(String[] args) throws Exception{
        DroneEngine engine = new DroneEngine();
        Position piscine = new Position(48.115367,-1.63781);
        Position croisement = new Position(48.11498, -1.63795);
        Position croisement2 = new Position(48.114454, -1.639962);
        Path path = new Path();
        path.addPosition(croisement);
        path.addPosition(croisement2);
        //engine.setPath(path);
        //engine.updateDrone();
    }
}
