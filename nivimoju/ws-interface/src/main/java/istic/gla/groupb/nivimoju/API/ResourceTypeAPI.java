package istic.gla.groupb.nivimoju.API;

import dao.ResourceTypeDAO;
import entity.ResourceType;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.util.List;

/**
 * Created by jules on 09/04/15.
 */
@Path("resource")
public class ResourceTypeAPI {

    /**
     * Gets all resource types
     * @return A list of resource types
     */
    @GET
    @Produces(MediaType.APPLICATION_JSON)
    public Response getResourceTypes() {
        ResourceTypeDAO resourceTypeDAO = new ResourceTypeDAO();
        resourceTypeDAO.connect();
        List<ResourceType> resourceTypes = resourceTypeDAO.getAll();
        resourceTypeDAO.disconnect();
        return Response.ok(resourceTypes).build();
    }

    /**
     * Gets resource types by ID
     * @return A resource type
     */
    @Path("/{idresource}")
    @GET
    @Produces(MediaType.APPLICATION_JSON)
    public Response getResourceTypeById(
            @PathParam("idresource") long idresource) {
        ResourceTypeDAO resourceTypeDAO = new ResourceTypeDAO();
        resourceTypeDAO.connect();
        ResourceType resourceType = resourceTypeDAO.getById(idresource);
        resourceTypeDAO.disconnect();
        return Response.ok(resourceType).build();
    }
}
