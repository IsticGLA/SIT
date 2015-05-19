/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package istic.gla.groupeb.flerjeco.codis.intervention;

import android.app.Activity;
import android.os.Build;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;

import org.apache.commons.collections4.CollectionUtils;
import org.apache.commons.collections4.ListUtils;

import java.util.ArrayList;
import java.util.List;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.adapter.InterventionAdapter;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;

public class InterventionFragment extends Fragment implements ISynchTool {
    OnResourceSelectedListener mCallback;

    private static final String TAG = InterventionFragment.class.getSimpleName();
    protected ListView listViewInterventions;

    // The container Activity must implement this interface so the frag can deliver messages
    public interface OnResourceSelectedListener {
        /** Called by HeadlinesFragment when a list item is selected */
        public void onResourceSelected(int position);
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState){
        super.onCreate(savedInstanceState);

        View v = inflater.inflate(R.layout.fragment_list_interventions_codis, container,
                false);

        listViewInterventions = (ListView) v.findViewById(R.id.listViewInterventions_codis);
        InterventionActivity interventionActivity = (InterventionActivity) getActivity();

        ArrayList<Intervention> interventions = new ArrayList<>();
        CollectionUtils.addAll(interventions, interventionActivity.getInterventions());
        InterventionAdapter adapter = new InterventionAdapter(interventionActivity ,interventions);

        listViewInterventions.setAdapter(adapter);
        listViewInterventions.setEmptyView(interventionActivity.findViewById(R.id.list_empty_interventions_view));

        listViewInterventions.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> adapterView, View view, int position, long id) {
                mCallback.onResourceSelected(position);
                listViewInterventions.setItemChecked(position, true);
                Log.i(TAG, "setOnItemClickListener : " + position);
            }
        });

        return v;
    }

    @Override
    public void onStart() {
        super.onStart();

        // When in two-pane layout, set the listview to highlight the selected list item
        // (We do this during onStart because at the point the listview is available.)
        if (getFragmentManager().findFragmentById(R.id.resources_fragment) != null) {
            listViewInterventions.setChoiceMode(ListView.CHOICE_MODE_SINGLE);
            listViewInterventions.setItemChecked(0, true);
            Log.i(TAG, "onStart setItemChecked");
        }
    }

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);

        // This makes sure that the container activity has implemented
        // the callback interface. If not, it throws an exception.
        try {
            mCallback = (OnResourceSelectedListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString()
                    + " must implement InterventionFragment.OnResourceSelectedListener");
        }
    }

    public void updateList() {
        Log.i(TAG, "updateList start");

        InterventionActivity interventionActivity = (InterventionActivity) getActivity();

        ArrayList<Intervention> interventions = new ArrayList<>();
        CollectionUtils.addAll(interventions, interventionActivity.getInterventions());
        InterventionAdapter adapter = new InterventionAdapter(interventionActivity ,interventions);

        listViewInterventions.setAdapter(adapter);
        listViewInterventions.setEmptyView(interventionActivity.findViewById(R.id.list_empty_interventions_view));
    }

    @Override
    public void refresh() {

    }
}