package istic.gla.groupb.nivimoju.API;

import entity.*;
import entity.Intervention;

import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.Produces;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.awt.*;

/**
 * Created by vivien on 13/04/15.
 */
@Path("notify")
public class Notify {
    /**
     * Gets all the interventions running
     * @return A list of interventions
     */
    @GET
    @Produces(MediaType.APPLICATION_JSON)
    public Response getInterventions() {
        Intervention intervention = new Intervention();
        intervention.setName("test");
        //Response.ok(intervention).build();
        return Response.noContent().build();
    }
}
