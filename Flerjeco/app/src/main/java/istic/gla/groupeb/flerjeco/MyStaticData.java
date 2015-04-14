package istic.gla.groupeb.flerjeco;

import android.app.Application;

import entity.StaticData;
import istic.gla.groupeb.flerjeco.springRest.SpringService;

/**
 * Created by erwann on 14/04/15.
 */
public class MyStaticData extends Application{

    private static MyStaticData singleInstance = null;
    private StaticData[] staticDatas;

    public static MyStaticData getSingleInstance() {
        return singleInstance;
    }

    @Override
    public void onCreate(){
        super.onCreate();
        SpringService springService = new SpringService();
        staticDatas = springService.getAllStaticDatas();
        singleInstance = this;
    }

    public StaticData[] getStaticDatas() {
        return staticDatas;
    }
}
