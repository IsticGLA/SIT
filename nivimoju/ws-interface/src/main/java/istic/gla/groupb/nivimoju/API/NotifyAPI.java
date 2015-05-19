package istic.gla.groupb.nivimoju.API;

import istic.gla.groupb.nivimoju.container.InterventionContainer;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.sql.Timestamp;
import java.util.Map;
import java.util.Stack;

/**
 * Created by vivien on 13/04/15.
 */
@Path("notify")
public class NotifyAPI {

    private Map<Integer, Stack<Response>> waitingRequests;

    /**
     * Gets all the interventions running
     * @return A list of interventions
     */
    @Path("intervention/{idIntervention}")
    @POST
    @Produces(MediaType.APPLICATION_JSON)
    @Consumes(MediaType.APPLICATION_JSON)
    public Response notifyIntervention(@PathParam("idIntervention") long idIntervention, Timestamp timestamp) {
        Timestamp databaseLastUpdate = InterventionContainer.getInstance().getLastUpdate(idIntervention);

        if (databaseLastUpdate != null && databaseLastUpdate.after(timestamp)) {
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
        Timestamp databaseLastUpdate = InterventionContainer.getInstance().getNewerLastUpdate();
        if (databaseLastUpdate.after(timestamp)) {
            // Content Different client / server
            return Response.status(201).entity(databaseLastUpdate).build();
        } else {
            return Response.ok().build();
        }
    }
}
