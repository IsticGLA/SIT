package istic.gla.groupb.nivimoju.API;

import dao.DroneDAO;
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
 * Created by jeremy on 11/04/15.
 */
@Path("drone")
public class Drone {

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
     * Find all drone affect to intervention of id idIntervention
     * @param idIntervention
     * @return
     */
    @GET
    @Path("/byIntervention/{idIntervention}")
    @Produces(MediaType.APPLICATION_JSON)
    public Response getAll(
            @PathParam("idIntervention") Long idIntervention) {
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connect();

        List<entity.Drone> droneList = droneDAO.getBy("idIntervention", idIntervention);
        droneDAO.disconnect();
        return Response.ok(droneList).build();
    }

    /**
     * Assign a drone to an intervention of id idIntervention
     * @param idIntervention
     * @return
     */
    @GET
    @Path("/assign/{idIntervention}")
    @Produces(MediaType.APPLICATION_JSON)
    public Response assign(
            @PathParam("idIntervention") Long idIntervention) {
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connect();

        List<entity.Drone> droneList = droneDAO.getBy("idIntervention", -1);

        if (null != droneList && droneList.size() > 1) {
            entity.Drone updateDrone = droneList.get(0);
            updateDrone.setIdIntervention(idIntervention);
            droneDAO.update(updateDrone);

            droneDAO.disconnect();
            return Response.ok(droneList.get(0)).build();
        } else {
            droneDAO.disconnect();
            return Response.status(Response.Status.NOT_FOUND).build();
        }
    }

    @POST
    @Path("/status/{droneNumber}")
    public Response move(
            @PathParam("droneNumber") int droneNumber) {
        DroneEngine engine = DroneEngine.getInstance();

        return Response.ok().build();
    }
}
