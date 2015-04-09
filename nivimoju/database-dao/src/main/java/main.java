import dao.UserDAO;
import entity.User;

import java.util.List;

/**
 * Created by vivien on 08/04/15.
 */
public class main {
    public static void main(String[] args) {
        UserDAO userDAO = new UserDAO();
        /*
        List<User> userList = userDAO.getAll();

        for(User user: userList) {
            System.out.println(user.getLogin());
            System.out.println(user.getPassword());
        }
        */
        User createUser = new User("login", "password");

        userDAO.connect();
        User user = userDAO.getById("1520530511913919500");
        userDAO.create(createUser);
        userDAO.disconnect();

        System.out.println(user.getLogin());
        System.out.println(user.getPassword());
    }
}
