import dao.UserDAO;
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
<<<<<<< Updated upstream

        /*User createUser = new User("login", "password");
=======
        */
        User createUser = new User("login2", "password2");
>>>>>>> Stashed changes

        userDAO.connect();
        User user = userDAO.getById("2");
        userDAO.create(createUser);
        userDAO.disconnect();

        System.out.println(user.getLogin());
        System.out.println(user.getPassword());*/
    }
}
