package istic.gla.groupeb.flerjeco.springRest;

import android.content.Context;

import istic.gla.groupb.nivimoju.entity.ResourceType;

/**
 * Created by jules on 18/05/15.
 */
public interface IResourceTypesActivity {
    void updateResourceTypes(ResourceType[] resourceTypes);

    Context getContext();
}
