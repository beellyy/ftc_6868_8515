package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FieldData {

    public static final String BEACON_BLUE_ROVER = "Blue-Rover";
    public static final String BEACON_RED_FOODPRINT = "Red-Footprint";
    public static final String BEACON_FRONT_CRATERS = "Front-Craters";
    public static final String BEACON_BACK_SPACE = "Back-Space";

    private FieldCoordinate beaconBlueRover = new FieldCoordinate(0,0);
    private FieldCoordinate beaconRedFootprint = new FieldCoordinate(0,0);
    private FieldCoordinate beaconFrontCraters = new FieldCoordinate(0,0);
    private FieldCoordinate beaconBackSpace = new FieldCoordinate(0,0);


    private Map<String,FieldCoordinate> fieldCoordinateMap = new HashMap<>();
    private Map<String,List<MoveAction>> fieldNavPath = new HashMap<>();

    public static FieldData fieldInstance = null;

    public FieldData(){

        initPoints();
        initActions();
    }

    private void initPoints() {
        fieldCoordinateMap.put(BEACON_BLUE_ROVER, beaconBlueRover);
        fieldCoordinateMap.put(BEACON_RED_FOODPRINT, beaconRedFootprint);
        fieldCoordinateMap.put(BEACON_FRONT_CRATERS, beaconFrontCraters);
        fieldCoordinateMap.put(BEACON_BACK_SPACE, beaconBackSpace);

        fieldCoordinateMap.put("Mineral_No_1",new FieldCoordinate(34.5,34.5));
        fieldCoordinateMap.put("Mineral_Center_1",new FieldCoordinate(34.5,34.5));
        fieldCoordinateMap.put("Mineral_Left_1",new FieldCoordinate(23.0,46.0));
        fieldCoordinateMap.put("Mineral_Right_1",new FieldCoordinate(46.0,23.0));

        fieldCoordinateMap.put("Mineral_No_2",new FieldCoordinate(-34.5,34.5));
        fieldCoordinateMap.put("Mineral_Center_2",new FieldCoordinate(-34.5,34.5));
        fieldCoordinateMap.put("Mineral_Left_2",new FieldCoordinate(-46.0,23.0));
        fieldCoordinateMap.put("Mineral_Right_2",new FieldCoordinate(-23.0,46.0));

        fieldCoordinateMap.put("Mineral_No_3",new FieldCoordinate(-34.5,-34.5));
        fieldCoordinateMap.put("Mineral_Center_3",new FieldCoordinate(-34.5,-34.5));
        fieldCoordinateMap.put("Mineral_Left_3",new FieldCoordinate(-23.0,-46.0));
        fieldCoordinateMap.put("Mineral_Right_3",new FieldCoordinate(-46.0,-23.0));

        fieldCoordinateMap.put("Mineral_No_4",new FieldCoordinate(34.5,-34.5));
        fieldCoordinateMap.put("Mineral_Center_4",new FieldCoordinate(34.5,-34.5));
        fieldCoordinateMap.put("Mineral_Left_4",new FieldCoordinate(46.0,-23.0));
        fieldCoordinateMap.put("Mineral_Right_4",new FieldCoordinate(23.0,46.0));

        fieldCoordinateMap.put("Storage_1",new FieldCoordinate(-57.5,57.5));
        fieldCoordinateMap.put("Storage_2",new FieldCoordinate(-57.5,57.5));
        fieldCoordinateMap.put("Storage_3",new FieldCoordinate(57.5,-57.5));
        fieldCoordinateMap.put("Storage_4",new FieldCoordinate(57.5,-57.5));

        fieldCoordinateMap.put("Crater_1",new FieldCoordinate(23.0,57.5));
        fieldCoordinateMap.put("Crater_2",new FieldCoordinate(-57.5,-22));
        fieldCoordinateMap.put("Crater_3",new FieldCoordinate(-23.0,-57.0));
        fieldCoordinateMap.put("Crater_4",new FieldCoordinate(57.5,22.0));
    }

    private void initActions(){
        List<MoveAction> actions = new ArrayList<>();
        actions.add(MoveAction.BuildBackwardAction(new FieldCoordinate(25,25)));
        actions.add(MoveAction.BuildTurnAction(135));
        actions.add(MoveAction.BuildForwardAction(new FieldCoordinate(-12,57.5)));
        actions.add(MoveAction.BuildTurnAction(180));
        actions.add(MoveAction.BuildForwardByWallAction(new FieldCoordinate(-57.5,57.5)));
        fieldNavPath.put("To_Storage_1",actions);
        actions = new ArrayList<>();
        actions.add(MoveAction.BuildBackwardAction(new FieldCoordinate(20,57.5)));
        fieldNavPath.put("To_Crater_1",actions);

        actions = new ArrayList<>();
        actions.add(MoveAction.BuildForwardAction(new FieldCoordinate(-57.5,57.5)));
        fieldNavPath.put("To_Storage_2",actions);
        actions = new ArrayList<>();
        actions.add(MoveAction.BuildTurnAction(270));
        actions.add(MoveAction.BuildBackwardAction(new FieldCoordinate(20,57.5)));
        actions.add(MoveAction.BuildForwardByWallAction(new FieldCoordinate(-57.5,-22)));
        fieldNavPath.put("To_Crater_2",actions);

        actions = new ArrayList<>();
        actions.add(MoveAction.BuildBackwardAction(new FieldCoordinate(-25,-25)));
        actions.add(MoveAction.BuildTurnAction(315));
        actions.add(MoveAction.BuildForwardAction(new FieldCoordinate(-12,-57.5)));
        actions.add(MoveAction.BuildTurnAction(0));
        actions.add(MoveAction.BuildForwardByWallAction(new FieldCoordinate(57.5,-57.5)));
        fieldNavPath.put("To_Storage_3",actions);
        actions = new ArrayList<>();
        actions.add(MoveAction.BuildBackwardAction(new FieldCoordinate(-20,-57.5)));
        fieldNavPath.put("To_Crater_3",actions);

        actions = new ArrayList<>();
        actions.add(MoveAction.BuildForwardAction(new FieldCoordinate(-57.5,57.5)));
        fieldNavPath.put("To_Storage_4",actions);
        actions = new ArrayList<>();
        actions.add(MoveAction.BuildTurnAction(90));
        actions.add(MoveAction.BuildBackwardAction(new FieldCoordinate(20,57.5)));
        actions.add(MoveAction.BuildForwardByWallAction(new FieldCoordinate(57.5,22)));
        fieldNavPath.put("To_Crater_4",actions);
    }

    public static FieldData defaultInstance(){
        if(null== fieldInstance){
            fieldInstance = new FieldData();
        }
        return fieldInstance;
    }

    public FieldCoordinate findBeacon(String name){
        return (fieldCoordinateMap.containsKey(name))? fieldCoordinateMap.get(name):FieldCoordinate.createInstance();
    }

    public FieldCoordinate findPoint(String name){
        return (fieldCoordinateMap.containsKey(name))? fieldCoordinateMap.get(name):FieldCoordinate.createInstance();
    }

    public List<MoveAction> findAction(String name){
        if(!fieldNavPath.containsKey(name)){
            return new ArrayList<>();
        }
        return fieldNavPath.get(name);

    }

    public double angle(FieldCoordinate posStart, String beacon, FieldCoordinate posTarget){

        FieldCoordinate posBeacon = findBeacon(beacon);
        double distST = posStart.distance(posTarget);
        double distSB = posStart.distance(posBeacon);
        double distBT = posBeacon.distance(posTarget);
        return (distST*distST+distSB*distSB-distBT*distBT)/ 2* distSB*distST;

    }


}
