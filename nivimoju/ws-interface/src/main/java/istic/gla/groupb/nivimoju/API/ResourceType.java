package istic.gla.groupb.nivimoju.API;

import dao.ResourceTypeDAO;

import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.Produces;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.util.List;

/**
 * Created by jules on 09/04/15.
 */
@Path("resource")
public class ResourceType {

    /**
     * Gets all resource types
     * @return A list of resource types
     */
    @GET
    @Produces(MediaType.APPLICATION_JSON)
    public Response getResourceTypes() {
        ResourceTypeDAO resourceTypeDAO = new ResourceTypeDAO();
        resourceTypeDAO.connect();
        List<entity.ResourceType> resourceTypes = resourceTypeDAO.getAll();
        entity.ResourceType resourceType = resourceTypeDAO.getById(new Long(1));
        resourceTypeDAO.disconnect();
        return Response.ok(resourceTypes).build();
    }
}
