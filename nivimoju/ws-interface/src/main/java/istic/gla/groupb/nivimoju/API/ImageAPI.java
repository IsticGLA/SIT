package istic.gla.groupb.nivimoju.API;

import istic.gla.groupb.nivimoju.builder.ImageBuilder;
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
        /*ImageDAO imageDAO = new ImageDAO();
        imageDAO.connect();
        Image result = imageDAO.getLastSpatialImage(null,null);
        imageDAO.disconnect();*/
        Image result = new Image();
        result.setBase64Image("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");
        result.setIdIntervention(1);
        double[] pos = {49.63004, -1.66334};
        result.setPosition(pos);
        result.setLastUpdate(new Timestamp(Long.parseLong("1432218236301")));
        result.setTimestamp(Long.parseLong("1432218234921"));
        result.setId(33);
        return Response.ok(result).build();
    }

    /**
     * Get all images (limited to a specific number) of the given coordinates for an intervention
     * @param inter The id of the intervention
     * @param lat The latitude coordinate of the image
     * @param lon The longitude coordinate of the image
     * @return OK
     */
    @GET
    @Path("/all/{inter}/{lat}/{lon}")
    @Produces(MediaType.APPLICATION_JSON)
    public Response getAllImages(@PathParam("inter") long inter, @PathParam("lat") double lat, @PathParam("lon") double lon) {
        logger.debug("getting image from database  " + lat + " " + lon);
        ImageDAO imageDAO = new ImageDAO();
        imageDAO.connect();
        double[] position = {lat, lon};
        List<Image> result = imageDAO.getLastSpatialImages(inter, position, 0L, 10);
        logger.info(String.format("got %d images for inter %d", result == null? 0 : result.size(), inter));
        imageDAO.disconnect();
        return Response.ok(result).build();
    }
}
