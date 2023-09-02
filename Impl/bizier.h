//
// Created by 28472 on 2023/8/14.
//

#ifndef AUTODRIVE_BIZIER_H
#define AUTODRIVE_BIZIER_H

#include <iostream>
#include <vector>
#include "SSD/SimPoint3D.h"


SSD::SimPoint3D
catmullRomInterpolation(SSD::SimPoint3D p0, SSD::SimPoint3D p1, SSD::SimPoint3D p2, SSD::SimPoint3D p3, double t) {
    double t2 = t * t;
    double t3 = t2 * t;

    double x = 0.5 * ((2.0 * p1.x) + (-p0.x + p2.x) * t + (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2 +
                      (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);

    double y = 0.5 * ((2.0 * p1.y) + (-p0.y + p2.y) * t + (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2 +
                      (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);

    SSD::SimPoint3D point0;
    point0.x = x;
    point0.y = y;
    point0.z = p0.z + p3.z;
    return point0;
}

void
ChangeLanePath(int num, SSD::SimPoint3D p0, SSD::SimPoint3D p1, SSD::SimPoint3D p2, SSD::SimPoint3D p3,
               SSD::SimPoint3DVector &path) {
    SSD::SimPoint3D point;
//    for (int i = 1; i <= num; i++) {
//        point = catmullRomInterpolation(p0, p1, p2, p3, i);
//        path.push_back(point);
//    }
    for(int i = 0; i < num; i++)
    {
        point.x=(p2.x-p1.x)*i/num+p1.x;
        point.y=(p2.y-p1.y)*i/num+p1.y;
        point.z=(p2.z-p1.z)*i/num+p1.z;;
        cout<<"point.x:"<<point.x<<"point.y:"<<point.y<<"point.z:"<<point.z<<endl;
        path.push_back(point);
    }

}

#endif //AUTODRIVE_BIZIER_H
