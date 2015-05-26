package istic.gla.groupb.nivimoju.API;

import istic.gla.groupb.nivimoju.dao.UserDAO;
import istic.gla.groupb.nivimoju.entity.User;

import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.PathParam;
import javax.ws.rs.core.Response;
import java.util.List;

/**
 * Created by jules on 08/04/15.
 */
@Path("authentication")
public class AuthenticationAPI {

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
        UserDAO userDAO = new UserDAO();
        userDAO.connect();

        List<User> userList = userDAO.getBy("login", user);

        if(userList != null) {
            for (User u : userList) {
                if (u.getPassword().equals(password)) {
                    userDAO.disconnect();
                    return Response.ok().build();
                }
            }
        }
        userDAO.disconnect();
        return Response.status(Response.Status.UNAUTHORIZED).build();
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
