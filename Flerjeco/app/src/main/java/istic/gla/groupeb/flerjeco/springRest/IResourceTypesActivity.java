package istic.gla.groupeb.flerjeco.springRest;

import android.content.Context;

import java.util.List;

import entity.Intervention;
import entity.ResourceType;

/**
 * Created by corentin on 16/04/15.
 */
public interface IResourceTypesActivity {
    void updateResourceTypes(ResourceType[] resourceTypes);

    Context getContext();
}
