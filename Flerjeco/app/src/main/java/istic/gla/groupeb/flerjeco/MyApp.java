package istic.gla.groupeb.flerjeco;

import android.app.Application;

/**
 * Created by corentin on 09/04/15.
 */
public class MyApp extends Application {

    private boolean isCodisUser; //make getter and setter
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
}
