package istic.gla.groupb.nivimoju.API;

import dao.DroneDAO;
import entity.Drone;
import entity.Intervention;
import entity.Position;
import istic.gla.goupb.nivimoju.drone.engine.DroneContainer;
import istic.gla.goupb.nivimoju.drone.engine.DroneEngine;
import istic.gla.groupb.nivimoju.drone.client.DroneClient;
import istic.gla.groupb.nivimoju.drone.latlong.LatLongConverter;
import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;
import org.apache.log4j.Logger;
import org.joda.time.DateTime;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.util.List;

/**
 * API to manipulate drones
 */
@Path("drone")
public class DroneAPI {
    Logger logger = Logger.getLogger(DroneAPI.class);


    /**
     * Return all drones
     * @return a response with all the drones
     */
    @GET
    @Produces(MediaType.APPLICATION_JSON)
    public Response getAll() {
        return Response.ok(DroneContainer.getInstance().getDrones())
                .build();
    }

    /**
     * Find all drone affected to intervention of id idIntervention
     * @param idIntervention the id of the intervention
     * @return a response with all the drones affected
     */
    @GET
    @Path("/byIntervention/{idIntervention}")
    @Produces(MediaType.APPLICATION_JSON)
    public Response getAllByIntervention(
            @PathParam("idIntervention") Long idIntervention) {
        return Response.ok(DroneContainer.getInstance().getDronesAssignedTo(idIntervention))
                .build();
    }

    /**
     * Unassign a drone from an intervention FOR DEBUG PURPOSES
     * @param label the id of the drone to unasign
     * @return ok
     */
    @GET
    @Path("/unassign/{label}")
    @Produces(MediaType.APPLICATION_JSON)
    public Response unassign(
            @PathParam("label") String label) {
        DroneContainer container = DroneContainer.getInstance();
        Drone drone = container.getDroneByLabel(label);
        container.freeDrone(drone);
        return Response.ok().build();
    }

    /**
     * alerte le droneEngine qu'une intervention a eu ses chemins mis à jours
     * @return
     */
    @POST
    @Path("/alertengine/")
    @Consumes(MediaType.APPLICATION_JSON)
    public Response alertEngine(Intervention intervention) {
        logger.info("alertEngine : received engine alert");
        if(intervention != null){
            logger.info("alertEngine : intervention = " + intervention.toString());
            logger.info("alertEngine start : " + DateTime.now());
                    DroneEngine.getInstance().computeForIntervention(intervention);
            logger.info("alertEngine end : " + DateTime.now());
            return Response.ok().build();
        }
        return Response.status(Response.Status.BAD_REQUEST)
                .build();
    }
}
