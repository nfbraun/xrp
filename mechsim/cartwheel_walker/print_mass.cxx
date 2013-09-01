#include <cstdio>
#include "StaticRobotInfo.h"

int main()
{
    printf("Pelvis:  %4.1f\n", rbMass(B_PELVIS));
    printf("Thigh:   %4.1f\n", rbMass(B_THIGH));
    printf("Shank:   %4.1f\n", rbMass(B_SHANK));
    printf("Foot:    %4.1f\n", rbMass(B_L_FOOT));
    printf("\n");
    printf("Total:   %4.1f\n", totalMass());
    
    return 0;
}
