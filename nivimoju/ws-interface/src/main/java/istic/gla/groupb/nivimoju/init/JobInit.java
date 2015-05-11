package istic.gla.groupb.nivimoju.init;

import istic.gla.goupb.nivimoju.drone.engine.DronePositionRefresherJob;
import org.apache.log4j.Logger;
import org.quartz.*;
import org.quartz.impl.StdSchedulerFactory;

import javax.servlet.ServletContextEvent;
import javax.servlet.ServletContextListener;

/**
 * Initialise le job qui rafraichit les positions de drones en lisant la simulation
 */
public class JobInit implements ServletContextListener {
    Logger logger = Logger.getLogger(JobInit.class);

    @Override
    public void contextInitialized(ServletContextEvent servletContextEvent) {
        logger.info("initializing context");
        JobDetail job = JobBuilder.newJob(DronePositionRefresherJob.class)
                .withIdentity("dummyJobName", "group1").build();

        Trigger trigger = TriggerBuilder
                .newTrigger()
                .withIdentity("dummyTriggerName", "group1")
                .withSchedule(
                        SimpleScheduleBuilder.simpleSchedule()
                                .withIntervalInMilliseconds(200)
                                .repeatForever())
                .build();

        try{
            Scheduler scheduler = new StdSchedulerFactory().getScheduler();
            scheduler.start();
            scheduler.scheduleJob(job, trigger);
        } catch (SchedulerException e){
            logger.error(e);
        }

    }

    @Override
    public void contextDestroyed(ServletContextEvent servletContextEvent) {

    }
}
