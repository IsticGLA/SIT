package istic.gla.groupeb.flerjeco;

import android.app.Application;
import android.util.Log;

import entity.StaticData;
import istic.gla.groupeb.flerjeco.springRest.SpringService;

/**
 * Created by erwann on 14/04/15.
 */
public class MyStaticData extends Application{

    private static final String TAG = MyStaticData.class.getSimpleName();
    private static MyStaticData singleInstance = null;
    private StaticData[] staticDatas;

    public static MyStaticData getSingleInstance() {
        return singleInstance;
    }

    @Override
    public void onCreate(){
        super.onCreate();
        Log.i(TAG, "Instanciating MyStaticData");
        singleInstance = this;
    }

    public StaticData[] getStaticDatas() {
        SpringService springService = new SpringService();
        staticDatas = springService.getAllStaticDatas();
        return staticDatas;
    }
}
