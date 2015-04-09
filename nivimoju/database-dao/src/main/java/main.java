import dao.IncidentCodeDAO;
import dao.InterventionDAO;
import dao.UserDAO;
import entity.Intervention;
import entity.Resource;
import entity.User;
import util.State;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by vivien on 08/04/15.
 */
public class main {
    public static void main(String[] args) {
        UserDAO userDAO = new UserDAO();
        userDAO.connect();
        List<User> userList = userDAO.getAll();
        userDAO.disconnect();
        for(User user: userList) {
            System.out.println(user.getLogin());
            System.out.println(user.getPassword());
        }

        User createUser = new User("login2", "password2");

        InterventionDAO interventionDAO = new InterventionDAO();

        userDAO.connect();
        User user = userDAO.getById(2L);
        userDAO.create(createUser);
        userDAO.disconnect();


        List<Resource> ressources = new ArrayList<>();
        ressources.add(new Resource("VSAV", State.planned));
        ressources.add(new Resource("VLCG", State.planned));
        Intervention inter = new Intervention(4, 48.11, -1.61, ressources, null, null, null, null);
        interventionDAO.connect();
        interventionDAO.create(inter);
        List<Intervention> intervention = interventionDAO.getAll();
        interventionDAO.disconnect();

        IncidentCodeDAO incidentCodeDAO = new IncidentCodeDAO();

        System.out.println(intervention.get(0).getLatitude());
        System.out.println(intervention.get(0).getResources().get(1));
    }
}
