// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import frc.robot.Constants.MKWEIGHTER;;

/** Add your docs here. */
public class Weigher {
    public ArrayList<GameObject[]>  grid;
    public Weigher()
    {
        grid = new ArrayList<GameObject[]>();
        for(int i = 0; i < 2; i++)
        {
            grid.add(new GameObject[8]);
            for(int j = 0; j < 8; j++)
            {
                double CONEORCUBE;
                double COOPERTITION;
                double POINTVALUE = MKWEIGHTER.POINTVALUE[i];
                if(j > 2 && j < 6)
                {
                    COOPERTITION = MKWEIGHTER.YESCOOPERTITION;
                }
                else
                {
                    COOPERTITION = MKWEIGHTER.NOCOOPERTITION;
                }
                if(j == 1 || j ==4 || j == 7)
                {
                    CONEORCUBE = MKWEIGHTER.CUBE;
                }
                else
                {
                    CONEORCUBE = MKWEIGHTER.CONE;
                }
                grid.get(i)[j] = new GameObject(MKWEIGHTER.EMPTY, MKWEIGHTER.DISTANCE[i][j], POINTVALUE, COOPERTITION, MKWEIGHTER.NOLINK, CONEORCUBE);
            }
        }
    }

    public class GameObject
    {
        public double empty;
        public double distance;
        public double pointValue;
        public double coopertition;
        public double link;
        public double coneOrCube;

        public GameObject(double empty, double distance, double pointValue, double coopertition, double link, double coneOrcube)
        {
            this.empty = empty;
            this.distance = distance;
            this.pointValue = pointValue;
            this.coopertition = coopertition;
            this.link = link;
            this.coneOrCube = coneOrcube;
        }
    }
}
