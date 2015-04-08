package istic.gla.groupb.nivimoju.API;

import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.PathParam;
import javax.ws.rs.core.Response;

/**
 * Created by jules on 08/04/15.
 */
@Path("authentication")
public class Authentication {

    /**
     * Request connection validation from the server
     * @param user The id of the user
     * @param password The password of the user
     * @return OK if authenticated
     */
    @GET
    @Path("/connected/{user}/{password}")
    public Response connect(
            @PathParam("user") String user,
            @PathParam("password") String password) {
        return Response.ok().build();
    }

    /**
     * Disconnect the user from the server
     * @param user The id of the user
     * @return
     */
    @GET
    @Path("/disconnected/{user}")
    public Response disconnect(@PathParam("user") String user) {
        return Response.ok().build();
    }
}
