package istic.gla.groupb.nivimoju.API;

import dao.DroneDAO;
import entity.Drone;
import entity.Position;
import istic.gla.goupb.nivimoju.drone.engine.DroneEngine;
import istic.gla.groupb.nivimoju.drone.client.DroneClient;
import istic.gla.groupb.nivimoju.drone.latlong.LatLongConverter;
import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.util.List;

/**
 * API to manipulate drones
 */
@Path("drone")
public class DroneAPI {

    /**
     * Send a move request directly to the simulation (for debug)
     * @param latitude The latitude of the position
     * @param longitude The longitude of the position
     * @return OK if authenticated
     */
    @GET
    @Path("/move/{lat}/{long}")
    public Response move(
            @PathParam("lat") double latitude,
            @PathParam("long") double longitude) {
        DroneClient client = new DroneClient();
        LatLongConverter converter = new LatLongConverter(48.1222, -1.6428, 48.1119, -1.6337, 720, 1200);
        Position position = new Position(latitude,longitude);

        LocalCoordinate local = converter.getLocal(position);
        local.setZ(20);
        try {
            client.postWaypoint(local);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return Response.ok().build();
    }

    /**
     * Find all drone affected to intervention of id idIntervention
     * @param idIntervention the id of the intervention
     * @return a response with all the drones affected
     */
    @GET
    @Path("/byIntervention/{idIntervention}")
    @Produces(MediaType.APPLICATION_JSON)
    public Response getAll(
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
}