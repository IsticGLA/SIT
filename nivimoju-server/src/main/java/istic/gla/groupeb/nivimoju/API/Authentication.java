package istic.gla.groupeb.nivimoju.API;

import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.PathParam;
import javax.ws.rs.core.Response;

/**
 * Created by jules on 08/04/15.
 */
@Path("/authentication")
public class Authentication {

    @GET
    @Path("/connected/{id}/{password}")
    public Response connect(@PathParam("id") String id, @PathParam("password") String password) {
        return Response.ok().build();
    }

    @GET
    @Path("/disconnected/{id}")
    public Response disconnect(@PathParam("id") String id) {
        return Response.ok().build();
    }
}
