package istic.gla.groupeb.flerjeco.springRest;

import android.content.Context;

import entity.StaticData;

/**
 * Created by jules on 16/04/15.
 */
public interface IStaticDataActivity {

    void setStaticData(StaticData[] data);

    Context getContext();
}
