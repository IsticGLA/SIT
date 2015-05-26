package istic.gla.groupb.nivimoju.container;

import istic.gla.groupb.nivimoju.entity.Image;

import java.util.HashMap;

/**
 * Ce container stocke la la derniere image de chaque drone, pour le flux "video"
 */
public class ImageContainer {
    private static ImageContainer instance;
    private HashMap<String, Image> mapImageByDroneLabel;

    /**
     * Initialise the DroneContainer with data from DB
     */
    private ImageContainer(){
        mapImageByDroneLabel = new HashMap<>();
    }

    public static synchronized ImageContainer getInstance(){
        if(instance == null){
            instance = new ImageContainer();
        }
        return instance;
    }

    /**
     * retourne la dernière image prise par le drone
     * @param droneLabel le label du drone
     * @return l'image ou null s'il n'y en a pas
     */
    public Image getLastImageOfDrone(String droneLabel){
        return mapImageByDroneLabel.get(droneLabel);
    }

    /**
     * set la dernière image prise par le drone
     * @param droneLabel le label du drone
     * @return l'image ou null s'il n'y en a pas
     */
    public Image setLastImageOfDrone(String droneLabel, Image image){
        return mapImageByDroneLabel.put(droneLabel, image);
    }
}
