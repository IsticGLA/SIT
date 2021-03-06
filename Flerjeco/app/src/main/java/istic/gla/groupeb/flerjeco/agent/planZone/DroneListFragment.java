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
package istic.gla.groupeb.flerjeco.agent.planZone;

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

import java.util.ArrayList;
import java.util.List;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Path;
import istic.gla.groupeb.flerjeco.R;

public class DroneListFragment extends Fragment {

    private static final String TAG = DroneListFragment.class.getSimpleName();

    // callback for the selected Listener
    OnResourceSelectedListener mCallback;

    // the listView of the fragment
    private ListView listViewPath;
    // Adapter for the ListView
    private ArrayAdapter adapter;
    // List of the label path in the current intervention
    private List<String> labelsPath;

    // The container Activity must implement this interface so the frag can deliver messages
    public interface OnResourceSelectedListener {
        /** Called by HeadlinesFragment when a list item is selected */
        public void onResourceSelected(int position);
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState){
        super.onCreate(savedInstanceState);

        View v = inflater.inflate(R.layout.fragment_list_drone, container, false);

        listViewPath = (ListView) v.findViewById(R.id.listViewPath);

        // We need to use a different list item layout for devices older than Honeycomb
        int layout = Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB ?
                android.R.layout.simple_list_item_activated_1 : android.R.layout.simple_list_item_1;

        // init of the label path list
        labelsPath = new ArrayList<>();
        List<Path> pathList = ((PlanZoneActivity) getActivity()).getIntervention().getWatchPath();
        for(int i = 0; i < pathList.size(); i++) {
            labelsPath.add("Trajet " + (i+1));
        }
        adapter = new ArrayAdapter<>(getActivity(), layout, labelsPath);

        // set adapter on the listView and set the listener
        listViewPath.setAdapter(adapter);
        listViewPath.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> adapterView, View view, int position, long id) {
                mCallback.onResourceSelected(position);
                listViewPath.setItemChecked(position, true);
            }
        });

        return v;
    }

    @Override
    public void onStart() {
        super.onStart();

        // When in two-pane layout, set the listview to highlight the selected list item
        // (We do this during onStart because at the point the listview is available.)
        PlanZoneMapFragment mapFragment = (PlanZoneMapFragment) getFragmentManager().findFragmentById(R.id.map_fragment);
        if (mapFragment != null) {
            listViewPath.setChoiceMode(ListView.CHOICE_MODE_SINGLE);
            listViewPath.setItemChecked(0,true);
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
                    + " must implement OnResourceSelectedListener");
        }
    }

    /**
     * refresh the listView when the update of the intervention is done
     * @param intervention
     */
    public void refresh(Intervention intervention){
        adapter.clear();
        labelsPath = new ArrayList<>();
        for(int i = 0; i < intervention.getWatchPath().size(); i++) {
            labelsPath.add("Trajet " + (i+1));
        }
        adapter.addAll(labelsPath);
        adapter.notifyDataSetChanged();
        listViewPath.setItemChecked(labelsPath.size()-1, true);
    }

    public void checkListView(int position){
        unCheckedListView();
        listViewPath.setItemChecked(position, true);
    }

    /**
     * unchecked the listView when you create a new path
     */
    public void unCheckedListView(){
        listViewPath.setItemChecked(listViewPath.getCheckedItemPosition(), false);
    }
}