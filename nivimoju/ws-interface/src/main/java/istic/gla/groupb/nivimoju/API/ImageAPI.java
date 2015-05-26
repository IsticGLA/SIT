package istic.gla.groupb.nivimoju.API;

import istic.gla.groupb.nivimoju.builder.ImageBuilder;
import istic.gla.groupb.nivimoju.container.ImageContainer;
import istic.gla.groupb.nivimoju.customObjects.TimestampedPosition;
import istic.gla.groupb.nivimoju.dao.ImageDAO;
import istic.gla.groupb.nivimoju.drone.FlaskImage;
import istic.gla.groupb.nivimoju.drone.engine.DroneEngine;
import istic.gla.groupb.nivimoju.entity.Image;
import org.apache.log4j.Logger;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.List;

/**
 * API to manipulate images
 */
@Path("image")
public class ImageAPI {
    Logger logger = Logger.getLogger(ImageAPI.class);

    /**
     * Creation of an Image entity to the database
     * @param flaskImage Image from flask to convert as an entity
     * @return OK
     */
    @POST
    @Path("/create")
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response createImage(FlaskImage flaskImage) {
        logger.debug("received image from flask...");
        Image image = new ImageBuilder().buildImage(flaskImage);
        /*BASE64Decoder decoder = new BASE64Decoder();
        try {
            byte[] img = new sun.misc.BASE64Decoder().decodeBuffer(image.getBase64Image());
            File outputfile = new File("/sit/log/test.jpeg");
            FileOutputStream osf = new FileOutputStream(outputfile);
            osf.write(img);
            osf.flush();
        } catch (IOException e) {
            logger.error("", e);
        }*/
        ImageDAO imageDAO = new ImageDAO();
        imageDAO.connect();
        Image result = imageDAO.addImage(image);
        logger.debug("Image inserted in database : " + result.getId());
        imageDAO.disconnect();
        return Response.ok(result).build();
    }

    /**
     * Get the last image of given coordinates for an intervention
     * @param inter The id of the intervention
     * @return OK
     */
    @GET
    @Path("/last/{inter}")
    @Produces(MediaType.APPLICATION_JSON)
    @Consumes(MediaType.APPLICATION_JSON)
    public Response getLastImage(@PathParam("inter") long inter, List<TimestampedPosition> timestampedPositionList) {
        logger.debug("getting image from database...");
        ImageDAO imageDAO = new ImageDAO();
        imageDAO.connect();
        long timestamp = -1;
        for (int i = 0; i < timestampedPositionList.size(); i++){
            long timestampTemp = timestampedPositionList.get(i).getTimestamp();
            if (timestamp == -1){
                timestamp = timestampTemp;
            } else if (timestamp > timestampTemp){
                timestamp = timestampTemp;
            }
        }
        List<Image> result = imageDAO.getAllLastSpatialImages(inter, timestamp, 10, timestampedPositionList);
        imageDAO.disconnect();
        return Response.ok(result).build();
    }

    /**
     * Get all images (limited to a specific number) of the given coordinates for an intervention
     * @param inter The id of the intervention
     * @param latitude The latitude coordinate of the image
     * @param longitude The longitude coordinate of the image
     * @param timestamp le timestamp de debut (on retourne les images prises apres ce timestamp)
     * @return OK
     */
    @GET
    @Path("/all/{inter}/{latitude}/{longitude}/{timestamp}")
    @Produces(MediaType.APPLICATION_JSON)
    public Response getAllImages(@PathParam("inter") long inter,
                                 @PathParam("latitude") double latitude,
                                 @PathParam("longitude") double longitude,
                                 @PathParam("timestamp") long timestamp) {
        logger.debug("getting image from database  " + latitude + " " + longitude + " timestamp:" + timestamp);
        ImageDAO imageDAO = new ImageDAO();
        imageDAO.connect();
        double[] position = {latitude, longitude};
        List<Image> result = imageDAO.getLastSpatialImages(inter, position, timestamp, 10, false);
        logger.info(String.format("got %d images for inter %d", result == null ? 0 : result.size(), inter));
        imageDAO.disconnect();
        return Response.ok(result).build();
    }

    /**
     * Creation of an Image entity in the container for video
     * @param flaskImage Image from flask to convert as an entity
     * @return OK
     */
    @POST
    @Path("/video")
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response createImageForVideo(FlaskImage flaskImage) {
        Image image = new ImageBuilder().buildImage(flaskImage);
        ImageContainer.getInstance().setLastImageOfDrone(flaskImage.getDroneLabel(), image);
        return Response.ok("image recorded").build();
    }

    /**
     * get the last image of a drone
     * @param droneLabel label of drone
     * @return OK
     */
    @GET
    @Path("/video/{droneLabel}")
    @Produces(MediaType.APPLICATION_JSON)
    public Response getImageForVideo(@PathParam("droneLabel") String droneLabel) {
        return Response
                .ok(ImageContainer.getInstance().getLastImageOfDrone(droneLabel))
                .build();
    }
}
