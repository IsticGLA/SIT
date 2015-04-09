package istic.gla.groupb.nivimoju.API;

import dao.IncidentCodeDAO;

import javax.ws.rs.*;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import java.util.List;

/**
 * Created by amhachi on 09/04/15.
 */
@Path("incidentCode")
public class IncidentCode {

    IncidentCodeDAO incidentCodeDAO ;

    /**
     * Gets all the incident codes
     * @return A list of incident codes
     */
    @Path("incidentCodesList")
    @GET
    @Produces({MediaType.APPLICATION_JSON})
    public List<entity.IncidentCode> getIncidentCodes() {
        incidentCodeDAO = new IncidentCodeDAO();
        return  incidentCodeDAO.getAll();
    }


}
