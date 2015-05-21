package istic.gla.groupb.nivimoju.API;

import istic.gla.groupb.nivimoju.builder.ImageBuilder;
import istic.gla.groupb.nivimoju.dao.ImageDAO;
import istic.gla.groupb.nivimoju.drone.FlaskImage;
import istic.gla.groupb.nivimoju.entity.Image;
import org.apache.log4j.Logger;
import sun.misc.BASE64Decoder;

import javax.imageio.ImageIO;
import javax.ws.rs.Consumes;
import javax.ws.rs.POST;
import javax.ws.rs.Path;
import javax.ws.rs.Produces;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileOutputStream;
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
        /*BASE64Decoder decoder = new BASE64Decoder();
        try {
            byte[] img = new sun.misc.BASE64Decoder().decodeBuffer(image.getImage());
            File outputfile = new File("/sit/log/test.jpeg");
            FileOutputStream osf = new FileOutputStream(outputfile);
            osf.write(img);
            osf.flush();
        } catch (IOException e) {
            logger.error("", e);
        }*/
        ImageDAO imageDAO = new ImageDAO();
        imageDAO.connect();
        //Image result = imageDAO.create(image);
        //logger.debug("Image inserted in database : " + result.getId());
        imageDAO.disconnect();
        return Response.ok(/*result*/).build();
    }
}
