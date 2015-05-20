package istic.gla.groupb.nivimoju.API;

import istic.gla.groupb.nivimoju.builder.ImageBuilder;
import istic.gla.groupb.nivimoju.dao.ImageDAO;
import istic.gla.groupb.nivimoju.drone.FlaskImage;
import istic.gla.groupb.nivimoju.entity.Image;
import org.apache.log4j.Logger;

import javax.imageio.ImageIO;
import javax.ws.rs.Consumes;
import javax.ws.rs.POST;
import javax.ws.rs.Path;
import javax.ws.rs.Produces;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

/**
 * API to manipulate images
 */
@Path("image")
public class ImageAPI {
    Logger logger = Logger.getLogger(ImageAPI.class);

    @POST
    @Path("/create")
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response createImage(FlaskImage flaskImage){
        logger.debug("received image from flask...");
        Image image = new ImageBuilder().buildImage(flaskImage);
        BufferedImage img = new BufferedImage((int)image.getWidth(), (int)image.getHeight(), BufferedImage.TYPE_INT_RGB);
        img.getRaster().setPixels(0,0,(int)image.getWidth(), (int)image.getHeight(), image.getImage());
        File outputfile = new File("/sit/log/test.bmp");
        try {
            ImageIO.write(img, "bmp", outputfile);
        } catch (IOException e) {
            logger.error("Error ", e);
        }
        logger.info("length of image received : " + flaskImage.getImage().length);
        logger.info("length of image built : " + image.getImage() == null ? 0 : image.getImage().length);
        ImageDAO imageDAO = new ImageDAO();
        imageDAO.connect();
        Image result = imageDAO.create(image);
        imageDAO.disconnect();
        return Response.ok(result).build();
    }
}
