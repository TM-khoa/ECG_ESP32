#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    HRServiceIdx = 0,
    // gán số thứ tự bắt buộc
    HRCharMeasureIdx = 1,// Characteristic Measurement index
    HRCharMeasureValIdx = 2,// Value of Characteristic Measurement index
    HRCCCIdx = 3, // Characteristic Client Configuration index

    HRCharCtrlPointIdx = 4,// Characteristic Heart Rate Control Point index
    HRCharCtrlPointValIdx = 5, // Value of Characteristic Heart Rate Control Point index

    HRTotalIdx = 6, // Total number of index of GATTS table
}HeartRate_GATTS_Table;

enum
{
    BATServiceIdx = 0,
    
    // gán số thứ tự bắt buộc
    BATCharLevelIdx = 1, // Characteristic Battery Level index
    BATCharLevelValIdx = 2, // Value of Characteristic Battery Level index
    BATCCCIdx = 3, // Characteristic Client Configuration index, 

    BATTotalIdx = 4, // Total number of index of GATTS table
}BATTERY_GATTS_Table;