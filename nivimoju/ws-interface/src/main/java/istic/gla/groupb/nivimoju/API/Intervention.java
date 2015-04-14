package istic.gla.groupb.nivimoju.API;

import dao.InterventionDAO;
import entity.Resource;
import util.State;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.util.List;

/**
 * Created by jules on 08/04/15.
 */
@Path("intervention")
public class Intervention {



    /**
     * Gets all the interventions running
     * @return A list of interventions
     */
    @GET
    @Produces(MediaType.APPLICATION_JSON)
    public Response getInterventions() {
        InterventionDAO interventionDAO = new InterventionDAO();

        interventionDAO.connect();
        List<entity.Intervention> inters = interventionDAO.getAll();
        interventionDAO.disconnect();
        return  Response.ok(inters).build();
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
            entity.Intervention intervention) {
        InterventionDAO interventionDAO= new InterventionDAO();
        interventionDAO.connect();
        entity.Intervention resultat = interventionDAO.create(intervention);
        interventionDAO.disconnect();
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
            entity.Intervention intervention) {
        InterventionDAO interventionDAO= new InterventionDAO();
        interventionDAO.connect();
        entity.Intervention result = interventionDAO.update(intervention);
        interventionDAO.disconnect();
        return  Response.ok(result).build();

    }

    /*

    /**
     * Creates a new intervention with a default list of vehicle in function of the sinister code
     * @param lat The latitude of the intervention
     * @param lng The longitude of the intervention
     * @param code The ID of the sinister code of the intervention
     * @return The id of the created intervention
     */
    /*
    @Path("{lat}/{lng}/{code}")
    @POST
    public Response createIntervention(
            @PathParam("lat") long lat,
            @PathParam("lng") long lng,
            @PathParam("code") int code) {


        interventionDAO.connect();
        entity.Intervention intervention = new entity.Intervention(code, lat, lng, null,null,null,null,null);
        entity.Intervention resultat = interventionDAO.create(intervention);
        System.out.println(resultat.getLatitude()+"/"+resultat.getLongitude());
        interventionDAO.disconnect();

        return Response.ok(resultat).build();
    }*/

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
     * @param res The type of the resource
     * @param oldState A String representing the old state
     * @param newState A String representing the wanted state
     * @return OK if the state has been correctly updated
     */
    @PUT
    @Path("{inter}/resources/{res}/{oldstate}/{newstate}")
    public Response changeResourceState(
            @PathParam("inter") Long inter,
            @PathParam("res") String res,
            @PathParam("oldstate") String oldState,
            @PathParam("newstate") String newState) {
        InterventionDAO interventionDAO = new InterventionDAO();
        interventionDAO.connect();
        entity.Intervention intervention = interventionDAO.getById(inter);
        try {
            for (entity.Resource resource : intervention.getResources()) {
                if (resource.getLabel().equals(res) && resource.getState().equals(oldState)) {
                    resource.setState(State.valueOf(newState));
                    break;
                }
            }
            interventionDAO.update(intervention);
            interventionDAO.disconnect();
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
        InterventionDAO interventionDAO = new InterventionDAO();
        interventionDAO.connect();
        entity.Intervention intervention = interventionDAO.getById(inter);
        intervention.getResources().add(new Resource(vehicle, State.waiting));
        interventionDAO.update(intervention);
        interventionDAO.disconnect();
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
