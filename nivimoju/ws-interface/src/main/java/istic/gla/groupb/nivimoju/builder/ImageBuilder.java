package istic.gla.groupb.nivimoju.builder;

import istic.gla.groupb.nivimoju.drone.FlaskImage;
import istic.gla.groupb.nivimoju.container.DroneContainer;
import istic.gla.groupb.nivimoju.drone.engine.DroneEngine;
import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupb.nivimoju.entity.Position;
import org.apache.log4j.Logger;
import org.joda.time.DateTime;

/**
 * Builder that converts FlaskImage to Image entity
 */
public class ImageBuilder {

    Logger logger = Logger.getLogger(ImageBuilder.class);

    public Image buildImage(FlaskImage flaskImage) {
        Drone drone = DroneContainer.getInstance().getDroneByLabel(flaskImage.getDroneLabel());
        if (drone != null) {
            Image result = new Image();
            result.setBase64Image(flaskImage.getBase64Image());
            Position position = DroneEngine.converter.getLatLong(flaskImage.getPosition());
            double[] posArray = {position.getLatitude(), position.getLongitude()};
            result.setPosition(posArray);
            result.setTimestamp(DateTime.now().getMillis());
            result.setIdIntervention(drone.getIdIntervention());
            return result;
        } else {
            logger.error("The drone does not seem to exist");
            return null;
        }
    }

}
