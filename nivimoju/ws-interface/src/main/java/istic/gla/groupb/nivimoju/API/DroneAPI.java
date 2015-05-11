package istic.gla.groupb.nivimoju.API;

import dao.DroneDAO;
import entity.Drone;
import entity.Intervention;
import entity.Position;
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
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connect();
        List<Drone> droneList = droneDAO.getAll();
        droneDAO.disconnect();
        return Response.ok(droneList).build();
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
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connect();
        List<Drone> droneList = droneDAO.getBy("idIntervention", idIntervention);
        droneDAO.disconnect();
        return Response.ok(droneList).build();
    }

    /**
     * Assign a drone to an intervention of id idIntervention
     * @param idIntervention the intervention that need a drone
     * @return OK everytime, with a body iff the assignation succeded
     */
    @GET
    @Path("/assign/{idIntervention}")
    @Produces(MediaType.APPLICATION_JSON)
    public Response assign(
            @PathParam("idIntervention") Long idIntervention) {
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connect();

        // on cherche un drone libre
        List<Drone> droneList = droneDAO.getBy("idIntervention", -1);

        if (null != droneList && droneList.size() >= 1) {
            Drone updateDrone = droneList.get(0);
            updateDrone.setIdIntervention(idIntervention);
            updateDrone.updateDate();
            droneDAO.update(updateDrone);
            droneDAO.disconnect();
            //notification au droneEngine
            DroneEngine.getInstance().assignDrone(updateDrone);
            return Response.ok(droneList.get(0)).build();
        } else {
            droneDAO.disconnect();
            return Response.ok().build();
        }
    }

    /**
     * Unassign a drone from an intervention
     * @param idDrone the id of the drone to unasign
     * @return ok
     */
    @GET
    @Path("/unassign/{idDrone}")
    @Produces(MediaType.APPLICATION_JSON)
    public Response unassign(
            @PathParam("idDrone") Long idDrone) {
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connect();

        // on cherche le drone
        Drone drone = droneDAO.getById(idDrone);

        if (null != drone) {
            drone.setIdIntervention(-1);
            drone.updateDate();
            droneDAO.update(drone);
            droneDAO.disconnect();
            //notification au droneEngine
            DroneEngine.getInstance().unasignDrone(drone);
            return Response.ok(drone).build();
        } else {
            droneDAO.disconnect();
            return Response.ok().build();
        }
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
                    DroneEngine.getInstance().setPathsForIntervention(intervention.getId(),
                            intervention.getWatchPath());
            logger.info("alertEngine end : " + DateTime.now());
            return Response.ok().build();
        }
        return Response.status(Response.Status.BAD_REQUEST)
                .build();
    }
}
