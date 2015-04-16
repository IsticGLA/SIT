package istic.gla.groupb.nivimoju.API;

import dao.InterventionDAO;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.sql.Timestamp;

/**
 * Created by vivien on 13/04/15.
 */
@Path("notify")
public class NotifyAPI {

    /**
     * Gets all the interventions running
     * @return A list of interventions
     */
    @Path("intervention/{idIntervention}")
    @POST
    @Produces(MediaType.APPLICATION_JSON)
    @Consumes(MediaType.APPLICATION_JSON)
    public Response notifyIntervention(@PathParam("idIntervention") long idIntervention, Timestamp timestamp) {
        InterventionDAO interventionDAO = new InterventionDAO();
        interventionDAO.connect();
        Timestamp databaseLastUpdate = interventionDAO.getLastUpdate(idIntervention);
        interventionDAO.disconnect();
        if (databaseLastUpdate.after(timestamp)) {
            // Content Different client / server
            return Response.status(201).entity(databaseLastUpdate).build();
        } else {
            return Response.ok().build();
        }
    }

    /**
     * Gets all the interventions running
     * @return A list of interventions
     */
    @Path("intervention")
    @POST
    @Produces(MediaType.APPLICATION_JSON)
    @Consumes(MediaType.APPLICATION_JSON)
    public Response notifyAllIntervention(Timestamp timestamp) {
        InterventionDAO interventionDAO = new InterventionDAO();
        interventionDAO.connect();
        Timestamp databaseLastUpdate = interventionDAO.getNewerLastUpdate();
        interventionDAO.disconnect();
        if (databaseLastUpdate.after(timestamp)) {
            // Content Different client / server
            return Response.status(201).entity(databaseLastUpdate).build();
        } else {
            return Response.ok().build();
        }
    }
}
