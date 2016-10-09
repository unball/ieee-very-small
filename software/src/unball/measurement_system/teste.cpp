/* 
 * @file   teste.cpp
 * @author Izabella Thais Oliveira Gomes
 * @date   01/10/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 * 
 * @brief Test rotation of the axis
 * 
 * This script is to test the method to rotate the axis, where the vision position datas are rotated +90 degrees related
 * to strategy position datas.
*/

#include <vector>
#include <cmath>
#include <iostream>
using namespace std;

void rotateAxis(float *teste)
{
    float aux;

    cout << endl << "before: " << teste[0] << "    " << teste[1] << "  " << teste[2] << endl;

    aux = -teste[0];
    teste[0] = teste[1];
    teste[1] = aux;
    teste[2] = (teste[2] > 0 ? teste[2] : (2*M_PI + teste[2])) * 360 / (2*M_PI); //convert to degrees
    cout << "degrees: " << teste[2] << endl;
    teste[2]  -= 90;
    cout << "degrees - 90: " << teste[2] << endl;
    if(teste[2] < 0)
        teste[2] += 360;
    cout << "degrees: " << teste[2] << endl;
    if(teste[2] == 90)
        teste[2] = M_PI_2;
    else if(teste[2] == 270)
        teste[2] = -M_PI_2;
    else
    {
        teste[2] = tan(teste[2] * M_PI/180.0);
        cout << "tg(degrees - 90): " << teste[2] << endl;
        teste[2] = atan2(teste[2], 1); // convert to radians
    }
    cout << "after: " << teste[0] << "    " << teste[1] << "  " << teste[2] << endl;
}

int main()
{
    float teste[3];
    
    cin >> teste[0];
    cin >> teste[1];
    cin >> teste[2];
    
    while(teste[0] != 1337)
    {
        rotateAxis(teste);
        cin >> teste[0];
        cin >> teste[1];
        cin >> teste[2];
    }

    return 0;
}