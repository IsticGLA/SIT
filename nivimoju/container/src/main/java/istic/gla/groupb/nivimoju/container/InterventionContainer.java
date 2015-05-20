package istic.gla.groupb.nivimoju.container;

import istic.gla.groupb.nivimoju.dao.InterventionDAO;
import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Path;
import istic.gla.groupb.nivimoju.entity.Resource;
import org.apache.log4j.Logger;
import istic.gla.groupb.nivimoju.util.State;

import java.sql.Timestamp;
import java.util.*;

/**
 * Aggregation class that store the interventions for quick access
 *
 * This class handles all modifications of interventions data and is the reference for every read operation
 */
public class InterventionContainer {
    Logger logger = Logger.getLogger(InterventionContainer.class);
    private static InterventionContainer instance;

    //the main container, used for read and write operations on interventions
    private Map<Long, Intervention> mapInterventionById;

    /**
     * Initialise the InterventionContainer with data from DB
     */
    private InterventionContainer(){
        mapInterventionById = new HashMap<>();
        logger.info("updating interventions from database");
        InterventionDAO interventionDAO = new InterventionDAO();
        interventionDAO.connect();
        List<Intervention> interventions = interventionDAO.getAll();
        interventionDAO.disconnect();
        if(interventions != null && interventions.size() > 0) {
            logger.info("got " + interventions.size() + " interventions from database");
            loadInterventions(interventions);
        } else{
            logger.error("failed to load interventions");
        }
    }

    /**
     * charge une liste d'intervention et prepare les maps interne
     * @param interventions la liste d'interventions a charger
     **/
    protected void loadInterventions(List<Intervention> interventions){
        logger.info("loading interventions internally");
        mapInterventionById.clear();
        for(Intervention intervention : interventions) {
            logger.info("loading intervention[" + intervention.getName() + "]");
            //update des listes interne
            mapInterventionById.put(intervention.getId(), intervention);
        }
        logger.info("loaded " + mapInterventionById.size() + " interventions : " + mapInterventionById.keySet());
    }

    protected Map<Long, Intervention> getMapInterventionById(){
        return mapInterventionById;
    }

    protected static void destroy(){
        instance = null;
    }

    /**
     * Accede a l'ensemble des interventions
     * @param intervention
     * @return Cree l'intervention intervention
     */
    public Intervention createIntervention(Intervention intervention){
        InterventionDAO interventionDAO= new InterventionDAO();
        interventionDAO.connect();

        int id =0;
        for(Resource res : intervention.getResources()){
            res.setIdRes(id);
            res.setLabel(res.getLabel() + id);
            id++;
        }

        Intervention resultat = interventionDAO.create(intervention);
        interventionDAO.disconnect();
        mapInterventionById.put(resultat.getId(), resultat);

        return intervention;
    }

    /**
     * Met un jour une intervention
     * @param intervention update l'intervention
     */
    public Intervention updateIntervention(Intervention intervention){
        Intervention interventionMap = mapInterventionById.get(intervention.getId());
        if(interventionMap != null) {
            intervention.updateDate();
            mapInterventionById.put(intervention.getId(), intervention);
        } else{
            logger.error("We got info for intervention [" + intervention.getId() +
                    "] but it does not seem to exist (existing ids : " + mapInterventionById.keySet() + ")");
        }
        return intervention;
    }

    /**
     * Change state of a ressource
     * @param id
     * @param res
     * @param state
     * @return
     */
    public Intervention changeResourceState(Long id, Long res, String state) {
        Intervention intervention = getInterventionById(id);
        for (Resource resource : intervention.getResources()) {
            if (resource.getIdRes() == res) {
                resource.setState(State.valueOf(state));
                break;
            }
        }
        intervention.updateDate();
        return intervention;
    }

    /**
     * Add Ressource to an intervention
     * @param idintervention
     * @param vehicleName
     * @return
     */
    public Intervention addResource(Long idintervention, String vehicleName) {
        Intervention intervention = getInterventionById(idintervention);
        Long id = Long.valueOf(0);
        for(Resource resource : intervention.getResources()) {
            if(resource.getIdRes() >= id) {
                id = resource.getIdRes();
            }
        }
        id++;
        intervention.getResources().add(new Resource(id, vehicleName + id, State.waiting));
        intervention.updateDate();
        return intervention;
    }

    /**
     * Add Ressource to an intervention
     * @param idintervention
     * @param vehicle
     * @return
     */
    public Intervention placeVehicle(Long idintervention, Resource vehicle) {
        Intervention intervention = getInterventionById(idintervention);
        for (Resource resource : intervention.getResources()) {
            if (resource.getIdRes() == vehicle.getIdRes()) {
                intervention.getResources().remove(resource);
                intervention.getResources().add(vehicle);
                return intervention;
            }
        }

        intervention = addResource(idintervention, vehicle.getLabel());

        intervention.updateDate();
        return intervention;
    }

    /**
     * Add Watchpath to an intervention
     * @param idintervention
     * @param path
     * @return
     */
    public Intervention addWatchPath(Long idintervention, Path path) {
        Intervention intervention = getInterventionById(idintervention);
        Long id = Long.valueOf(0);
        for(Path oldpath : intervention.getWatchPath()) {
            if(oldpath.getIdPath() >= id) {
                id = oldpath.getIdPath();
            }
        }
        id++;
        path.setIdPath(id);
        intervention.getWatchPath().add(path);
        intervention.updateDate();
        return intervention;
    }

    /**
     * Update Watchpath to an intervention
     * @param idintervention
     * @param path
     * @return
     */
    public Intervention updateWatchPath(Long idintervention, Path path) {
        Intervention intervention = getInterventionById(idintervention);
        Long id = Long.valueOf(0);
        for(Path oldpath : intervention.getWatchPath()) {
            if(oldpath.getIdPath() == path.getIdPath()) {
                intervention.getWatchPath().remove(oldpath);
                intervention.getWatchPath().add(path);
                intervention.updateDate();
                return intervention;
            }
        }
        return intervention;
    }

    /**
     * Delete Watchpath to an intervention
     * @param idintervention
     * @param path
     * @return
     */
    public Intervention deleteWatchPath(Long idintervention, Path path) {
        Intervention intervention = getInterventionById(idintervention);
        Long id = Long.valueOf(0);
        for(Path oldpath : intervention.getWatchPath()) {
            if(oldpath.getIdPath() == path.getIdPath()) {
                intervention.getWatchPath().remove(oldpath);
                intervention.updateDate();
                return intervention;
            }
        }
        return intervention;
    }

    public static synchronized InterventionContainer getInstance(){
        if(instance == null){
            instance = new InterventionContainer();
        }
        return instance;
    }

    /**
     * Accede a l'ensemble des interventions
     * @return toutes les interventions
     */
    public Collection<Intervention> getInterventions(){
        return mapInterventionById.values();
    }

    /**
     * Accede a l'ensemble des interventions
     * @param id
     * @return l'intervention d'id id
     */
    public Intervention getInterventionById(Long id){
        return mapInterventionById.get(id);
    }

    /**
     * return the LastUpdate from the container
     * @param id
     * @return
     */
    public Timestamp getLastUpdate(long id) {
        Intervention intervention = getInterventionById(id);
        if(intervention != null) {
            return intervention.getLastUpdate();
        }
        return null;
    }

    /**
     * return the Newer LastUpdate from a type in the database
     * @return
     */
    public Timestamp getNewerLastUpdate() {
        Timestamp maxTimestamp = new Timestamp(0);
        Timestamp timestamp = null;
        for(Intervention inter : getInterventions()) {
            timestamp= inter.getLastUpdate();
            if(maxTimestamp.before(timestamp)) {
                maxTimestamp = timestamp;
            }
        }
        return maxTimestamp;
    }
}
