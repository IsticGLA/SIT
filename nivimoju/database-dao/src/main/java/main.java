import dao.IncidentCodeDAO;
import entity.IncidentCode;

import java.util.List;

/**
 * Created by vivien on 08/04/15.
 */
public class main {
    public static void main(String[] args) {
        IncidentCodeDAO dao = new IncidentCodeDAO();
        dao.connect();
        List<IncidentCode> l = dao.getAll();
        for (IncidentCode i : l){
            System.out.println(i.getCode() + "  " + i.getresourceType());
        }

    }
}
