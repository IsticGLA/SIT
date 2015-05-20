package istic.gla.groupb.nivimoju.API;

import istic.gla.groupb.nivimoju.container.DroneContainer;
import istic.gla.groupb.nivimoju.container.InterventionContainer;
import istic.gla.groupb.nivimoju.drone.engine.DroneEngine;
import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Resource;
import org.apache.log4j.Logger;

import javax.ws.rs.*;
import javax.ws.rs.Path;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.util.Collection;

/**
 * Created by jules on 08/04/15.
 */
@Path("intervention")
public class InterventionAPI {
    Logger logger = Logger.getLogger(InterventionAPI.class);

    /**
     * Gets all the interventions running
     * @return A list of interventions
     */
    @GET
    @Produces(MediaType.APPLICATION_JSON)
    public Response getInterventions() {
        Collection<Intervention> inters = InterventionContainer.getInstance().getInterventions();
        logger.trace("intervention:" + inters);
        return  Response.ok(inters).build();
    }

    /**
     * Gets Intervention by ID
     * @return An intervention
     */
    @Path("/{idintervention}")
    @GET
    @Produces(MediaType.APPLICATION_JSON)
    public Response getInterventionById(
            @PathParam("idintervention") long idintervention) {
        Intervention intervention = InterventionContainer.getInstance().getInterventionById(idintervention);
        return Response.ok(intervention).build();
    }

    /**
     * Creates a new intervention with a default list of vehicle in function of the sinister code
     * @param intervention
     * @return The id of the created intervention
     */
    @Path("/create")
    @POST
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response createIntervention(
            Intervention intervention) {
        Intervention resultat = InterventionContainer.getInstance().createIntervention(intervention);
        return  Response.ok(resultat).build();
    }

    /**
     * Update an intervention
     * @param intervention
     * @return The id of the updated intervention
     */
    @Path("/update")
    @POST
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response updateIntervention(
            Intervention intervention) {
        Intervention result = InterventionContainer.getInstance().updateIntervention(intervention);
        return  Response.ok(result).build();

    }

    /**
     * Stops the intervention
     * @param inter The id of the intervention
     * @return OK if the intervention has been correctly stopped
     */
    @PUT
    @Path("{inter}/stopped")
    public Response stopIntervention(@PathParam("inter") String inter) {
        return Response.ok().build();
    }

    /**
     * Gets all the agents of an intervention
     * @param inter The id of the intervention
     * @return A list of agents
     */
    @GET
    @Path("{inter}/agent")
    public Response getAgents(@PathParam("inter") String inter) {
        return Response.ok().build();
    }

    /**
     * Adds an agent on an intervention
     * @param inter The id of the intervention
     * @param agent The id of the agent
     * @return OK if the agent has been correctly added to the intervention
     */
    @PUT
    @Path("{inter}/agent/{agent}")
    public Response addAgent(
            @PathParam("inter") String inter,
            @PathParam("agent") String agent) {
        return Response.ok().build();
    }

    /**
     * Gets all the resources of an intervention
     * @param inter The id of the intervention
     * @return A list of resources
     */
    @GET
    @Path("{inter}/resources")
    public Response getResources(@PathParam("inter") String inter) {
        return Response.ok().build();
    }

    /**
     * Changes the state of a resource
     * @param inter The id of the intervention
     * @param res The id of the resource
     * @param state A String representing the state
     * @return OK if the state has been correctly updated
     */
    @PUT
    @Path("{inter}/resources/{res}/{state}")
    public Response changeResourceState(
            @PathParam("inter") Long inter,
            @PathParam("res") Long res,
            @PathParam("state") String state) {
        try {
            Intervention intervention = InterventionContainer.getInstance().changeResourceState(inter, res, state);
            return Response.ok(intervention).build();
        } catch (Exception ex) {
            return Response.serverError().build();
        }
    }

    /**
     * Requests a vehicle for the intervention
     * @param inter The id of the intervention
     * @param vehicle the label of the requested vehicle type
     * @return The id of the requested vehicle
     */
    @PUT
    @Path("{inter}/resources/{vehicle}")
    public Response requestVehicle(
            @PathParam("inter") Long inter,
            @PathParam("vehicle") String vehicle) {
        Intervention intervention = InterventionContainer.getInstance().addResource(inter, vehicle);
        return Response.ok(intervention).build();
    }

    /**
     * Places the vehicle at coordinates with a role
     * @param inter The id of the intervention
     * @param newResource resource
     * @return OK if the vehicle has been correctly placed
     */
    @POST
    @Path("{inter}/resources/update")
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response placeVehicle(
            Resource newResource,
            @PathParam("inter") Long inter) {
        Intervention intervention = InterventionContainer.getInstance().placeVehicle(inter, newResource);
        return Response.ok(intervention).build();
    }

    /**
     * Add the watchPath
     * @param inter The id of the intervention
     * @param newPath resource
     * @return OK
     */
    @POST
    @Path("{inter}/watchpath/create")
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response newWatchpath(
            @PathParam("inter") Long inter,
            istic.gla.groupb.nivimoju.entity.Path newPath) {
        Intervention intervention = InterventionContainer.getInstance().addWatchPath(inter, newPath);

        requestOrFreeDrone(intervention);

        return Response.ok(intervention).build();
    }

    /**
     * create a path for an intervention and assign a drone to it
     * @return CREATED if successful, SERVICE_UNAVAILABLE if no drone is available, NOT_FOUND if the intervention does not exist
     */
    @POST
    @Path("{inter}/watchpath/update")
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response updatePaths(@PathParam("inter") Long inter, istic.gla.groupb.nivimoju.entity.Path path) {
        logger.info("updating path for intervention " + inter);

        Intervention oldInter = InterventionContainer.getInstance().getInterventionById(inter);
        if(oldInter == null){
            logger.warn("intervention does not seem to exist in db");
            return Response.status(Response.Status.NOT_FOUND)
                    .build();
        }

        Intervention intervention = InterventionContainer.getInstance().updateWatchPath(inter, path);

        requestOrFreeDrone(intervention);

        return  Response.status(Response.Status.OK)
                .entity(intervention)
                .build();
    }

    /**
     * Delete the watchPath
     * @param inter The id of the intervention
     * @param newPath resource
     * @return OK
     */
    @POST
    @Path("{inter}/watchpath/delete")
    @Consumes(MediaType.APPLICATION_JSON)
    @Produces(MediaType.APPLICATION_JSON)
    public Response deleteWatchpath(
            @PathParam("inter") Long inter,
            istic.gla.groupb.nivimoju.entity.Path newPath) {
        Intervention intervention = InterventionContainer.getInstance().deleteWatchPath(inter, newPath);

        requestOrFreeDrone(intervention);

        return Response.ok(intervention).build();
    }

    /**
     * Request free drone
     * @param intervention
     */
    public void requestOrFreeDrone(Intervention intervention) {
        //request or free drones
        int neededDroneNumber = intervention.getWatchPath().size() + intervention.getWatchArea().size();
        int currentlyAssignedDroneNumber =
                DroneContainer.getInstance().getDronesAssignedTo(intervention.getId()).size();
        int deltaDroneNumber = neededDroneNumber - currentlyAssignedDroneNumber;
        if(deltaDroneNumber > 0) {
            logger.info(String.format("the path update is asking for %d new drones", deltaDroneNumber));
            DroneContainer.getInstance().requestDrones(intervention.getId(), deltaDroneNumber);
        } else if(deltaDroneNumber < 0) {
            logger.info(String.format("the path update is asking for liberation of %d drones", -deltaDroneNumber));
            DroneContainer.getInstance().freeDrones(intervention.getId(), -deltaDroneNumber);
        } else {
            logger.info("the path update does not need to change drone affectations");
        }

        //alerting the engine
        DroneEngine.getInstance().computeForIntervention(intervention);
    }
}
