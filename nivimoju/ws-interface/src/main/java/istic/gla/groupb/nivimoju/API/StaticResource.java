package istic.gla.groupb.nivimoju.API;

import dao.StaticDataDAO;
import entity.StaticData;

import javax.ws.rs.GET;
import javax.ws.rs.Path;
import javax.ws.rs.Produces;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.util.List;

/**
 * Created by erwann on 13/04/15.
 */
@Path("staticdata")
public class StaticResource {

    /**
     * Get all the static datas
     * @return a list of static data
     */
    @GET
    @Produces(MediaType.APPLICATION_JSON)
    public Response getStaticDatas(){
        StaticDataDAO staticDataDAO = new StaticDataDAO();

        staticDataDAO.connect();
        List<StaticData> datas = staticDataDAO.getAll();
        staticDataDAO.disconnect();

        return Response.ok(datas).build();
    }

}
