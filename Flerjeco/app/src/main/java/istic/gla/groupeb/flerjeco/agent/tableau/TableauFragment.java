package istic.gla.groupeb.flerjeco.agent.tableau;

import android.app.Activity;
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

import istic.gla.groupeb.flerjeco.R;

/**
 * A simple {@link Fragment} subclass.
 * Activities that contain this fragment must implement the
 * {@link TableauFragment.OnFragmentInteractionListener} interface
 * to handle interaction events.
 * Use the {@link TableauFragment#newInstance} factory method to
 * create an instance of this fragment.
 */
public class TableauFragment extends Fragment {
    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";

    // TODO: Rename and change types of parameters
    private String mParam1;
    private String mParam2;

    private OnFragmentInteractionListener mListener;

    /**
     * Use this factory method to create a new instance of
     * this fragment using the provided parameters.
     *
     * @param param1 Parameter 1.
     * @param param2 Parameter 2.
     * @return A new instance of fragment TableauFragment.
     */
    // TODO: Rename and change types and number of parameters
    public static TableauFragment newInstance(String param1, String param2) {
        TableauFragment fragment = new TableauFragment();
        Bundle args = new Bundle();
        args.putString(ARG_PARAM1, param1);
        args.putString(ARG_PARAM2, param2);
        fragment.setArguments(args);
        return fragment;
    }

    public TableauFragment() {
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

        // Recuperation du table layout sur lequel nous allons agir
        String[] moyen = {"Moyen","Demandé à", "Déclenché à", "Arrivé à", "Envoyé à", "Libéré à"};

        // On va calculer la largeur des colonnes en fonction de la marge de 10
        // On affiche l'enreg dans une ligne
        TableRow tableRow = new TableRow(getActivity());
        containerTable.addView(tableRow,
                new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));
        containerTable.setBackgroundColor(getResources().getColor(R.color.grey));

        // On crée une ligne de x moyen colonnes
        tableRow.setLayoutParams(new LayoutParams(moyen.length));

        // On va commencer par renseigner une ligne de titre par joueur
        int i = 0;
        for (String player : moyen) {
            TextView text = createTextView(false , i == moyen.length - 1);
            text.setText(player);
            text.setGravity(Gravity.CENTER);
            tableRow.addView(text, i++);
        }

        for (int j = 0; j < 10; j++) {
            tableRow = new TableRow(getActivity());

            containerTable.addView(tableRow,
                    new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));
            i = 0;
            for (String player : moyen) {
                TextView text = createTextView(j==9, i == moyen.length - 1);
                text.setText("123");
                text.setTextColor(getResources().getColor(R.color.red));
                tableRow.addView(text, i++);
                text.setGravity(Gravity.RIGHT);
            }
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
        return inflater.inflate(R.layout.fragment_tableau2, container, false);
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

}
