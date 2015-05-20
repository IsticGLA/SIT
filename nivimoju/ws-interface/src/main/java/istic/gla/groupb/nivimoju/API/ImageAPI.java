package istic.gla.groupb.nivimoju.API;

import istic.gla.groupb.nivimoju.API.flask.FlaskImage;
import istic.gla.groupb.nivimoju.builder.ImageBuilder;
import istic.gla.groupb.nivimoju.dao.ImageDAO;
import istic.gla.groupb.nivimoju.entity.Image;

import javax.ws.rs.Consumes;
import javax.ws.rs.POST;
import javax.ws.rs.Path;
import javax.ws.rs.Produces;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;

/**
 * API to manipulate images
 */
@Path("image")
public class ImageAPI {

    @POST
    @Path("/create")
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response createImage(FlaskImage flaskImage){
        ImageDAO imageDAO = new ImageDAO();
        imageDAO.connect();
        Image result = imageDAO.create(new ImageBuilder().buildImage(flaskImage));
        imageDAO.disconnect();
        return Response.ok(result).build();
    }
}
