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
        IncidentCodeDAO dao = new IncidentCodeDAO();
        dao.connect();

        List<IncidentCode> l = dao.getAll();

        for (IncidentCode code : l){
            System.out.println("Code : "+code.getCode());
            for (Long idRT : code.getresourceType()){
                System.out.println("Id resource : "+idRT);
            }
        }

        dao.disconnect();



    }
}
