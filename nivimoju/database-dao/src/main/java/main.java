import dao.IncidentCodeDAO;
import dao.InterventionDAO;
import entity.IncidentCode;
import entity.Intervention;

import java.util.List;

/**
 * Created by vivien on 08/04/15.
 */
public class main {
    public static void main(String[] args) {
        InterventionDAO dao = new InterventionDAO();
        dao.connect();
        Intervention intervention = new Intervention(1,23.2,32.2,null,null,null,null,null);
        Intervention res = dao.create(intervention);
        System.out.println(res.getLongitude()+"/"+res.getLatitude());

        dao.disconnect();


    }
}
