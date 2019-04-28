package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.FieldCoordinate;

/**
 * 机器人执行的动作
 * */
public class MoveAction {


    /**
     * 动作类别
     */
    public enum ActionType{

        /**
         * 按当前方向向前
         */
        Forward,

        /**
         * 按当前方向向前，如果探测到墙，则溜墙根前进
         */
        ForwardByWall,

        /**
         * 后退
         */
        Backward,

        /**
         * 转到指定角度
         */
        Turn
    }

    public ActionType actionType= ActionType.Forward;
    // 用于 forward 和 backard 代表距离，用于 turn 代表基于场地坐标系的角度
    public FieldCoordinate data = null;

    public MoveAction(){
        data = new FieldCoordinate();
    }

    public static MoveAction BuildForwardAction(FieldCoordinate data){
        MoveAction action = new MoveAction();
        action.actionType = ActionType.Forward;
        action.data = data;
        return action;
    }
    public static MoveAction BuildForwardByWallAction(FieldCoordinate data){
        MoveAction action = new MoveAction();
        action.actionType = ActionType.ForwardByWall;
        action.data = data;
        return action;
    }
    public static MoveAction BuildBackwardAction(FieldCoordinate data){
        MoveAction action = new MoveAction();
        action.actionType = ActionType.Backward;
        action.data = data;
        return action;
    }
    public static MoveAction BuildTurnAction(double angle){
        MoveAction action = new MoveAction();

        action.actionType = ActionType.Turn;
        action.data.x = angle;
        return action;
    }


}
