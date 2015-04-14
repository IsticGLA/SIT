package istic.gla.groupeb.flerjeco;

import android.app.Application;

import entity.StaticData;
import istic.gla.groupeb.flerjeco.springRest.SpringService;

/**
 * Created by corentin on 09/04/15.
 */
public class MyApp extends Application {

    private boolean isCodisUser; //make getter and setter
    private String login, password;
    private StaticData[] staticDatas;
    private static MyApp singleInstance = null;

    public static MyApp getInstance()
    {
        return singleInstance;
    }

    @Override
    public void onCreate() {
        super.onCreate();
        singleInstance = this;
    }

    public boolean isCodisUser() {
        return isCodisUser;
    }

    public void setCodisUser(boolean isCodisUser) {
        this.isCodisUser = isCodisUser;
    }

    public String getPassword() {
        return password;
    }

    public void setPassword(String password) {
        this.password = password;
    }

    public String getLogin() {
        return login;
    }

    public void setLogin(String login) {
        this.login = login;
    }

    public StaticData[] getStaticDatas() {
        return staticDatas;
    }

    public void setStaticDatas(StaticData[] staticDatas) {
        this.staticDatas = staticDatas;
    }
}
