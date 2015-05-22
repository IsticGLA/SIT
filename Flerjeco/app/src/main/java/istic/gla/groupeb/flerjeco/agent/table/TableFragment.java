package istic.gla.groupeb.flerjeco.agent.table;

import android.app.Activity;
import android.graphics.Color;
import android.net.Uri;
import android.os.Bundle;
import android.support.v4.app.*;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TableLayout;
import android.widget.TableRow;
import android.widget.TableRow.LayoutParams;
import android.widget.TextView;

import java.sql.Timestamp;
import java.text.DecimalFormat;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Resource;
import istic.gla.groupb.nivimoju.util.ResourceCategory;
import istic.gla.groupb.nivimoju.util.ResourceRole;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.synch.ISynchTool;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;

/**
 * A simple {@link Fragment} subclass.
 * Activities that contain this fragment must implement the
 * {@link TableFragment.OnFragmentInteractionListener} interface
 * to handle interaction events.
 * Use the {@link TableFragment#newInstance} factory method to
 * create an instance of this fragment.
 */
public class TableFragment extends Fragment implements ISynchTool{
    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";

    // TODO: Rename and change types of parameters
    private String mParam1;
    private String mParam2;

    private OnFragmentInteractionListener mListener;



    //Manipulated intervention
    private Intervention intervention;
    /**
     * Use this factory method to create a new instance of
     * this fragment using the provided parameters.
     *
     * @param param1 Parameter 1.
     * @param param2 Parameter 2.
     * @return A new instance of fragment TableauFragment.
     */
    // TODO: Rename and change types and number of parameters
    public static TableFragment newInstance(String param1, String param2) {
        TableFragment fragment = new TableFragment();
        Bundle args = new Bundle();
        args.putString(ARG_PARAM1, param1);
        args.putString(ARG_PARAM2, param2);
        fragment.setArguments(args);
        return fragment;
    }

    public TableFragment() {
        // Required empty public constructor
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (getArguments() != null) {
            mParam1 = getArguments().getString(ARG_PARAM1);
            mParam2 = getArguments().getString(ARG_PARAM2);
        }
    }

    private TableLayout containerTable ;



    @Override
    public void onActivityCreated(Bundle savedInstanceState) {
        super.onActivityCreated(savedInstanceState);
        containerTable = (TableLayout) getActivity().findViewById(R.id.containerTable);

        refresh();
    }

    /**
     * Method that changes the function of the vehicle
     * @param function the new function of the vehicle
     */
    public int getResourceColor(ResourceRole function){

        switch (function){
            case water:
                return Color.BLUE;

            case fire:
                return Color.RED;

            case people:
                return Color.rgb(102,255,102);

            case risks:
                return Color.rgb(255,102,0);

            case commands:
                return Color.rgb(153,0,102);

            default:
                return Color.BLACK;

        }
    }


    private TextView createTextView(boolean endline, boolean endcolumn){
        TextView text = new TextView(getActivity(), null, R.style.frag2HeaderCol);
        int bottom = endline ? 1 : 0;
        int right = endcolumn ? 1 :0;
        LayoutParams params = new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.MATCH_PARENT, 0.3f);
        params.setMargins(1, 1, right, bottom);
        text.setLayoutParams(params);
        text.setPadding(4, 4, 10, 4);
        text.setBackgroundColor(getResources().getColor(R.color.white));
        return text;
    }
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        return inflater.inflate(R.layout.fragment_tableau, container, false);
    }

    // TODO: Rename method, update argument and hook method into UI event
    public void onButtonPressed(Uri uri) {
        if (mListener != null) {
            mListener.onFragmentInteraction(uri);
        }
    }

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        try {
            mListener = (OnFragmentInteractionListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString()
                    + " must implement OnFragmentInteractionListener");
        }
    }

    @Override
    public void onDetach() {
        super.onDetach();
        mListener = null;
    }

    /**
     * This interface must be implemented by activities that contain this
     * fragment to allow an interaction in this fragment to be communicated
     * to the activity and potentially other fragments contained in that
     * activity.
     * <p/>
     * See the Android Training lesson <a href=
     * "http://developer.android.com/training/basics/fragments/communicating.html"
     * >Communicating with Other Fragments</a> for more information.
     */
    public interface OnFragmentInteractionListener {
        // TODO: Update argument type and name
        public void onFragmentInteraction(Uri uri);
    }

    public TableLayout getContainerTable() {
        return containerTable;
    }

    public void setContainerTable(TableLayout containerTable) {
        this.containerTable = containerTable;
    }

    @Override
    public void refresh() {
        //Delete all content before painting in containerTable
        containerTable.removeAllViews();

        String[] moyen = null;
        try {
            moyen = getResources().getStringArray(R.array.resourceDateState);
        } catch (IllegalStateException ex) { return; }

        // get intervention
        intervention = ((TableActivity) getActivity()).getIntervention();

        TableRow tableRow = new TableRow(getActivity());
        tableRow.setLayoutParams(new LayoutParams(moyen.length));
        if (intervention.getResources().size()>0) {
            for (final Resource resource : intervention.getResources()) {
                if (ResourceCategory.vehicule.equals(resource.getResourceCategory())) {
                    tableRow = new TableRow(getActivity());

                    containerTable.addView(tableRow,
                            new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));
                    int i = 0;
                    for (int j = 0; j < moyen.length; j++) {
                        TextView text = createTextView(true, true);

                        Timestamp timestamp;

                        //Number formatter for minutes ##00
                        DecimalFormat decimalFormat = new DecimalFormat("##00");
                        switch (j) {
                            case 0:
                                text.setText(resource.getLabel());
                                break;
                            case 1:
                                timestamp = resource.getWaitingHistory();
                                if (timestamp != null) {
                                    text.setText( decimalFormat.format(timestamp.getHours())  + decimalFormat.format(timestamp.getMinutes()));
                                } else {
                                    text.setText("-");
                                }
                                break;
                            case 2:
                                timestamp = resource.getValidatedHistory();
                                if (timestamp != null) {
                                    text.setText( decimalFormat.format(timestamp.getHours())  + decimalFormat.format(timestamp.getMinutes()));
                                } else {
                                    text.setText("-");
                                }
                                break;
                            case 3:
                                timestamp = resource.getArrivedHistory();
                                if (timestamp != null) {
                                    text.setText( decimalFormat.format(timestamp.getHours())  + decimalFormat.format(timestamp.getMinutes()));
                                } else {
                                    text.setText("-");
                                }
                                break;
                            case 4:
                                timestamp = resource.getPlannedHistory();
                                if (timestamp != null) {
                                    text.setText( decimalFormat.format(timestamp.getHours())  + decimalFormat.format(timestamp.getMinutes()));
                                } else {
                                    text.setText("-");
                                }
                                break;
                            case 5:
                                timestamp = resource.getFreeHistory();
                                if (timestamp != null) {
                                    text.setText( decimalFormat.format(timestamp.getHours())  + decimalFormat.format(timestamp.getMinutes()));
                                } else {
                                    text.setText("-");
                                }
                                break;
                            default:
                                text.setText("-");
                                break;
                        }
                        text.setWidth(100);
                        text.setTextColor(getResourceColor(resource.getResourceRole()));
                        tableRow.addView(text, i++);
                        text.setGravity(Gravity.CENTER);
                    }
                }
            }
        }

    }

    @Override
    public void onPause() {
        super.onPause();
        IntentWraper.stopService();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
    }

    @Override
    public void onLowMemory() {
        super.onLowMemory();
    }
}
