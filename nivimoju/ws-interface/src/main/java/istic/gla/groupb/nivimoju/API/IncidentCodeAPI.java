package istic.gla.groupb.nivimoju.API;

import dao.IncidentCodeDAO;
import entity.IncidentCode;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.util.List;

/**
 * Created by amhachi on 09/04/15.
 */
@Path("incidentcode")
public class IncidentCodeAPI {

    IncidentCodeDAO incidentCodeDAO ;

    /**
     * Gets all the incident codes
     * @return A list of incident codes
     */
    @GET
    @Produces(MediaType.APPLICATION_JSON)
    public Response getIncidentCodes() {
        incidentCodeDAO = new IncidentCodeDAO();
        incidentCodeDAO.connect();
        List<IncidentCode> listCodes = incidentCodeDAO.getAll();
        incidentCodeDAO.disconnect();
        return Response.ok(listCodes).build();
    }


}
