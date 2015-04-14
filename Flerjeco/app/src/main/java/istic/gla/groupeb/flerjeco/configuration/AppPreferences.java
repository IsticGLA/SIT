package istic.gla.groupeb.flerjeco.configuration;

import android.os.Bundle;
import android.preference.PreferenceActivity;

import istic.gla.groupeb.flerjeco.R;


public class AppPreferences extends PreferenceActivity  {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        addPreferencesFromResource(R.xml.preferences);
    }
}
