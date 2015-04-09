import dao.InterventionDAO;
import dao.UserDAO;
import entity.Intervention;
import entity.User;

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
        User user = userDAO.getById("2");
        userDAO.create(createUser);
        userDAO.disconnect();

        interventionDAO.connect();
        List<Intervention> intervention = interventionDAO.getAll();
        interventionDAO.disconnect();

        System.out.println(intervention.get(0).getLatitude());
        System.out.println(intervention.get(0).getResources().get(1));
    }
}
