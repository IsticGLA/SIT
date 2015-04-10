package istic.gla.groupb.nivimoju.API;

import dao.InterventionDAO;

import javax.ws.rs.*;
import javax.ws.rs.core.Response;

/**
 * Created by jules on 08/04/15.
 */
@Path("intervention")
public class Intervention {

    InterventionDAO interventionDAO;

    /**
     * Gets all the interventions running
     * @return A list of interventions
     */
    @GET
    public Response getInterventions() {
        return Response.ok().build();
    }

    /**
     * Creates a new intervention with a default list of vehicle in function of the sinister code
     * @param lat The latitude of the intervention
     * @param lng The longitude of the intervention
     * @param code The sinister code of the intervention
     * @return The id of the created intervention
     */
    @POST
    @Path("{lat}/{lng}/{code}")
    public Response createIntervention(
            @PathParam("lat") long lat,
            @PathParam("lng") long lng,
            @PathParam("code") String code) {


        interventionDAO.connect();
        entity.Intervention intervention = new entity.Intervention();
        interventionDAO.disconnect();

        return Response.ok().build();
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
     * @param state A String representing the wanted state
     * @return OK if the state has been correctly updated
     */
    @PUT
    @Path("{inter}/resources/{res}/{state}")
    public Response changeResourceState(
            @PathParam("inter") String inter,
            @PathParam("res") String res,
            @PathParam("state") String state) {
        return Response.ok().build();
    }

    /**
     * Requests a vehicle for the intervention
     * @param inter The id of the intervention
     * @param vehicle A String representing the type of vehicle requested
     * @return The id of the requested vehicle
     */
    @PUT
    @Path("{inter}/resources/{vehicle}")
    public Response requestVehicle(
            @PathParam("inter") String inter,
            @PathParam("vehicle") String vehicle) {
        return Response.ok().build();
    }

    /**
     * Places the vehicle at coordinates with a role
     * @param inter The id of the intervention
     * @param res The id of the resource
     * @param lat The latitude of the vehicle
     * @param lng The longitude of the vehicle
     * @param role A String representing the role of the vehicle
     * @return OK if the vehicle has been correctly placed
     */
    @PUT
    @Path("{inter}/resources/{res}/{lat}/{lng}/{role}")
    public Response placeVehicle(
            @PathParam("inter") String inter,
            @PathParam("res") String res,
            @PathParam("lat") long lat,
            @PathParam("lng") long lng,
            @PathParam("role") String role) {
        return Response.ok().build();
    }
}
