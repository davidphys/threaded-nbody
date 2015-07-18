#include "easytime.h"

typedef std::chrono::time_point<std::chrono::system_clock> TimeType;
typedef std::chrono::duration<double> DurationType;

namespace easytime{
    TimeType getPresent(){
        return std::chrono::system_clock::now();
    }
}

EasyTimer::EasyTimer(){
	timer=std::chrono::system_clock::now();
}
EasyTimer::~EasyTimer(){}
double EasyTimer::tick(){
	TimeType timer2=std::chrono::system_clock::now();
	DurationType duration=timer2-timer;
	timer=timer2;
	return duration.count(); 
}
