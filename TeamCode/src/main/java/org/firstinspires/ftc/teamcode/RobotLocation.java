package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.FieldCoordinate;

public class RobotLocation {
    public double x = 0.0;
    public double y = 0.0;
    public double heading = 0.0;
    public String targetName = "";
    public boolean found = false;
    public FieldCoordinate coordinate = new FieldCoordinate();


    /**
     * 获取当前点所在象限
     * 第1象限中的点： x>0 y>0
     * 第2象限中的点： x<0 y>0
     * 第3象限中的点： x<0 y<0
     * 第4象限中的点： x>0 y<0
     * */
    public int quadrant() {
        if(x<0 && y<0){
            return 3;
        }
        if(x<0 && y>0){
            return 2;
        }
        if(x>0 && y<0){
            return 4;
        }

        return 1;
    }

    /**
     * 获取当前点所在象限
     * 第1象限中的点： x>0 y>0
     * 第2象限中的点： x<0 y>0
     * 第3象限中的点： x<0 y<0
     * 第4象限中的点： x>0 y<0
     * */
    public FieldCoordinate coordinate() {
        coordinate.x = this.x;
        coordinate.y = this.y;
        return coordinate;
    }
}
