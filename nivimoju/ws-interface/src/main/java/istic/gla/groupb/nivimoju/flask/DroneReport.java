package istic.gla.groupb.nivimoju.flask;

import entity.Position;
import istic.gla.goupb.nivimoju.drone.engine.DroneEngine;
import istic.gla.groupb.nivimoju.API.Drone;
import istic.gla.groupb.nivimoju.drone.client.DroneClient;

import javax.ws.rs.GET;
import javax.ws.rs.POST;
import javax.ws.rs.Path;
import javax.ws.rs.PathParam;
import javax.ws.rs.core.Response;

/**
 * Created by sacapuces on 14/04/15.
 */
@Path("drone/report")
public class DroneReport {

    @POST
    @Path("/status/{droneNumber}")
    public Response move(
            @PathParam("droneNumber") int droneNumber) {
        DroneEngine engine = DroneEngine.getInstance();

        return Response.ok().build();
    }
}
