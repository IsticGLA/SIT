package istic.gla.groupb.nivimoju.API;

import dao.InterventionDAO;
import entity.Intervention;
import entity.Resource;
import org.apache.log4j.Logger;
import util.State;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.util.List;

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
        InterventionDAO interventionDAO = new InterventionDAO();

        interventionDAO.connect();
        List<Intervention> inters = interventionDAO.getAll();
        logger.info("intervention:"+inters);
        interventionDAO.disconnect();
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
        InterventionDAO interventionDAO = new InterventionDAO();
        interventionDAO.connect();
        Intervention intervention = interventionDAO.getById(idintervention);
        interventionDAO.disconnect();
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
        InterventionDAO interventionDAO= new InterventionDAO();
        interventionDAO.connect();
        Intervention resultat = interventionDAO.create(intervention);
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
            Intervention intervention) {
        InterventionDAO interventionDAO= new InterventionDAO();
        interventionDAO.connect();
        Intervention result = interventionDAO.update(intervention);
        interventionDAO.disconnect();
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
        InterventionDAO interventionDAO = new InterventionDAO();
        interventionDAO.connect();
        Intervention intervention = interventionDAO.getById(inter);
        try {
            for (Resource resource : intervention.getResources()) {
                if (resource.getIdRes() == res) {
                    resource.setState(State.valueOf(state));
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
        Intervention intervention = interventionDAO.getById(inter);
        Long id = Long.valueOf(0);
        for(Resource resource : intervention.getResources()) {
            if(resource.getIdRes() >= id) {
                id = resource.getIdRes();
            }
        }
        id++;
        intervention.getResources().add(new Resource(id, vehicle + id, State.waiting));
        intervention = interventionDAO.update(intervention);
        interventionDAO.disconnect();
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
        boolean found = false;
        InterventionDAO interventionDAO = new InterventionDAO();
        interventionDAO.connect();
        Intervention intervention = interventionDAO.getById(inter);
        for (Resource resource : intervention.getResources()) {
            if (resource.getIdRes() == newResource.getIdRes()) {
                resource = newResource;
                found = true;
            }
        }
        if (!found){
            intervention.getResources().add(newResource);
        }
        intervention.updateDate();
        intervention = interventionDAO.update(intervention);
        interventionDAO.disconnect();
        return Response.ok(intervention).build();
    }
}
