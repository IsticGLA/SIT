package istic.gla.groupb.nivimoju.API;

import dao.InterventionDAO;
import entity.*;
import entity.Intervention;

import javax.swing.text.html.parser.Entity;
import javax.ws.rs.*;
import javax.ws.rs.Path;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.awt.*;
import java.sql.Timestamp;

/**
 * Created by vivien on 13/04/15.
 */
@Path("notify")
public class Notify {

    /**
     * Gets all the interventions running
     * @return A list of interventions
     */
    @Path("/{idIntervention}")
    @POST
    @Produces(MediaType.APPLICATION_JSON)
    @Consumes(MediaType.APPLICATION_JSON)
    public Response notifyIntervention(@PathParam("idIntervention") long idIntervention, Timestamp timestamp) {
        InterventionDAO interventionDAO = new InterventionDAO();
        interventionDAO.connect();
        Timestamp databaseLastUpdate = interventionDAO.getLastUpdate(idIntervention, timestamp);
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
    @POST
    @Produces(MediaType.APPLICATION_JSON)
    @Consumes(MediaType.APPLICATION_JSON)
    public Response notifyAllIntervention(Timestamp timestamp) {
        InterventionDAO interventionDAO = new InterventionDAO();
        interventionDAO.connect();
        Timestamp databaseLastUpdate = interventionDAO.getNewerLastUpdate(timestamp);
        interventionDAO.disconnect();
        if (databaseLastUpdate.after(timestamp)) {
            // Content Different client / server
            return Response.status(201).entity(databaseLastUpdate).build();
        } else {
            return Response.ok().build();
        }
    }
}
